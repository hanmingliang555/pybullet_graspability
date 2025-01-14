import time
import numpy as np
import math
import pybullet as p
import trimesh
import pybullet_data
import json
import argparse
import pandas as pd
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
import io
import torch
import os


def run():
    log = pd.DataFrame(columns=['抓取结果', '掉落质量(kg)','空间角变化(弧度)','机械臂移动距离','末端执行器位置','机械臂位置','物体位置', '物体质心位置','接触点位置','法线','距离','正向接触力','侧向摩擦力','相对于物体质心的距离','在物体坐标系中的坐标'])
    object,constraint=create_object()
    #计算从物体坐标系到世界坐标系的变化矩阵，4*4的，包括旋转和平移
    object_T_global=get_object_matrix(object,center_of_mass)
    #移除物体，以防机械臂运动过程中碰撞
    p.removeBody(object)
    flags=1
    #依次读取json文件内容进行仿真
    for i in range(100):
        #读取json文件，获得从夹爪坐标系到物体坐标系的变换矩阵，4x4,包括旋转和平移,flag_1代表着文件内是否还有有效数据
        grasp_center,grasp_endpoints,gravity,flag_1=read_json(obj_key,i)
        gravity_setting(gravity)
        # 初始化一行数据为 None
        log.loc[len(log)] = [None] * len(log.columns)
        #初始化设置log
        log=initialization_log(log)
        if flag_1==True:
            for j in range(num_points):
                if flags==1 or flags==-1:
                    gripper_basePosition=gripper_pos_list[j]
                    #gripper_basePosition=args.position
                panda=create_gripper(gripper_basePosition)
                #执行仿真,falgs=-1说明机械臂位置不对，flags=False说明该姿势抓不住，flags=True说明可以抓住物体
                flags,panda,object,log=step(panda,object_T_global,grasp_center,grasp_endpoints,log,i,center_of_mass)
                #逐渐增加物体质量，返回最后掉落时的质量
                mass=increase_mass(flags,panda,object)
                print(mass)
                #移除所有物体、机械臂、辅助线
                if object==-1:
                    p.removeBody(panda)
                    p.removeAllUserDebugItems()
                else:
                    p.removeBody(panda)
                    p.removeBody(object)
                    p.removeAllUserDebugItems()
                #记录抓取成功时的信息
                if flags==True or (flags == False and mass == -1):
                    #print(gripper_basePosition)
                    if flags==True:
                        log.iloc[-1, 0] = '抓取成功'
                    log.iloc[-1, 1] = mass
                    log.at[log.index[-1], "机械臂位置"] = gripper_basePosition
                    log.at[log.index[-1],'物体位置'] = object_position
                    log.to_csv(f'{output_path}.csv', index=True,na_rep='NA')
                    break
                log.to_csv(f'{output_path}.csv', index=True,na_rep='NA')
            #这里的flags已经从寻找位置的for循环跳出，如果falgs=-1,说明没找到合适位置
            if flags==-1:
                log.iloc[-1, 0] = '未找到抓取位置'
                flags=1
        else :
            print("数据读取完毕，仿真结束")
            break

# 加载机械臂，将它的位置固定在gripper_basePosition
def create_gripper(gripper_basePosition):
    panda = p.loadURDF("simple_gripper.urdf",
                    basePosition=gripper_basePosition, useFixedBase=True)
    p.changeDynamics(panda, linkIndex=-1, lateralFriction=friction_gripper)
    p.createConstraint(parentBodyUniqueId=panda,
                    parentLinkIndex=-1,
                    childBodyUniqueId=-1,
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=gripper_basePosition,
                    childFramePosition=[0, 0, 0])
    return panda

#创建一个物体，固定它的位置在start_position
def create_object():
    # 创建一个可视化形状，使用 Mesh 文件加载物体
    visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                       fileName=mesh_path,
                                       rgbaColor=[1, 1, 1, 0.7],  # 设置颜色
                                       meshScale=mesh_scale)
    # 创建碰撞形状（通常需要为网格创建近似的碰撞形状）
    collision_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                             fileName=mesh_path,
                                             meshScale=mesh_scale)
    #z_axis_neg_up_orientation = p.getQuaternionFromEuler([math.pi, 0, 0])  # 绕 x 轴旋转 180°
    # 创建物体并设置初始位置和姿态
    object = p.createMultiBody(baseMass=mass,  # 设置质量
                               baseCollisionShapeIndex=collision_shape,
                               baseVisualShapeIndex=visual_shape,
                               baseInertialFramePosition=center_of_mass,
                               basePosition=object_position,   # 物体位置
                               #baseOrientation=z_axis_neg_up_orientation)
                               baseOrientation=[0, 0, 0, 1])
    # 通过创建约束的方式固定物体在object_position
    constraint=p.createConstraint(parentBodyUniqueId=object,
                   parentLinkIndex=-1,  # 基座固定
                   childBodyUniqueId=-1,  # 没有子物体
                   childLinkIndex=-1,
                   jointType=p.JOINT_FIXED,  # 固定约束
                   jointAxis=[0, 0, 0],
                   parentFramePosition=list(map(lambda x: -x, center_of_mass)),
                   childFramePosition=object_position)  # 固定位置在object_position
    #修改物体摩擦系数
    p.changeDynamics(object, linkIndex=-1, lateralFriction=friction_object,spinningFriction=0.3)
    return object,constraint

#创建一个球体，用来表示质心位置
def create_sphere(target_position,radius,rgb):
    # 创建球体的视觉形状
    sphere_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,          # 选择球体作为形状类型
        radius=radius,                       # 设置球体的半径
        rgbaColor=rgb            # 设置颜色 (红色，RGBA)
    )

    # 创建一个多体对象，将视觉和碰撞形状组合
    sphere = p.createMultiBody(
        baseMass=0,                       # 物体的质量
        baseCollisionShapeIndex=-1,  # 碰撞形状
        baseVisualShapeIndex=sphere_visual_shape,        # 视觉形状
        basePosition=target_position            # 设置球体的位置
    )
    return sphere

#使用相机进行每个姿势的截图
def carmer_photo(i,grasp_center,left_finger_target,right_finger_target,target_position):

    up_vector = [0, 0, 1]  # 相机的"上"向量
    normal=calculate_normal_from_points(grasp_center,left_finger_target,right_finger_target)
    camera_position_1 = target_position+0.3*normal
    camera_position_2 = target_position-0.3*normal
    # 设置相机参数
    view_matrix_1 = p.computeViewMatrix(camera_position_1, target_position, up_vector)
    view_matrix_2 = p.computeViewMatrix(camera_position_2, target_position, up_vector)
    projection_matrix = p.computeProjectionMatrixFOV(fov=45, aspect=1.0, nearVal=0.1, farVal=100.0)

    width = 640
    height = 480
    img_arr_1 = p.getCameraImage(width, height, viewMatrix=view_matrix_1, projectionMatrix=projection_matrix)
    img_arr_2 = p.getCameraImage(width, height, viewMatrix=view_matrix_2, projectionMatrix=projection_matrix)
        
    # 获取图像数据
    rgb_img_1 = np.array(img_arr_1[2])
    rgb_img_2 = np.array(img_arr_2[2])
    # 保存图像
    image_pil_1 = Image.fromarray(rgb_img_1)
    image_pil_2 = Image.fromarray(rgb_img_2)
    # 创建绘图对象
    draw_1 = ImageDraw.Draw(image_pil_1)
    draw_2 = ImageDraw.Draw(image_pil_2)

    # 添加文字到左上角
    text_1 = f'{output_path}_{i}_1.png'
    text_2 = f'{output_path}_{i}_2.png' 
    text_position = (10, 10)  # 文字位置，左上角 (x, y)
    draw_1.text(text_position, text_1, fill="black")  # 在第一张图片上添加文字
    draw_2.text(text_position, text_2, fill="black")  # 在第二张图片上添加文字

    image_pil_1.save(f'{output_path}_{i}_1.png')
    image_pil_2.save(f'{output_path}_{i}_2.png')

#将夹爪坐标系里的点映射到世界坐标系里
def gripperpoint_T_global(gripper_point,gripper_T_object,object_T_global):
    #将原坐标后面加一个1,以便于计算
    gripper_point_new=np.hstack((gripper_point,1))
    #将夹爪坐标系点映射到物体坐标系
    gripper_point_new=np.dot(gripper_point_new,gripper_T_object.T)
    #将物体坐标系的点映射到世界坐标系
    gripper_point_new=np.dot(gripper_point_new,object_T_global.T)
    #只取前三个坐标
    gripper_point_new = gripper_point_new[:3]
    return gripper_point_new

#将物体坐标系里的点映射到世界坐标系
def objectpoint_T_global(object_point,object_T_global):
    #将原坐标后面加一个1,以便于计算
    object_point_new=np.hstack((object_point,1))
    #将物体坐标系的点映射到世界坐标系
    object_point_new=np.dot(object_point_new,object_T_global.T)
    #只取前三个坐标
    object_point_new = object_point_new[:3]
    return object_point_new

#将一个向量转化为四元数
def gripper_ori_T_rotation_quaternion(target_ori):

    # 归一化
    target_ori = target_ori / np.linalg.norm(target_ori)  
    # 默认抓取方向 (即Z轴方向) [0, 0, 1]
    default_gravity_ori = np.array([0, 0, 1])

    # 计算旋转轴（通过叉积获得）
    rotation_axis = np.cross(default_gravity_ori, target_ori)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)  # 归一化
    # 计算旋转角度（通过点积获得）
    cos_theta = np.dot(default_gravity_ori, target_ori)
    theta = np.arccos(cos_theta)  # 旋转角度
    # 使用旋转轴和旋转角度来创建四元数
    rotation_quaternion = p.getQuaternionFromAxisAngle(rotation_axis, theta)
    return rotation_quaternion

#计算两个四元数的角度误差
# 计算四元数之间的旋转角度误差
def quaternion_error(q1, q2):
    # q1, q2 是四元数，应该是单位四元数，格式为 [w, x, y, z]
    
    # 转换四元数为旋转对象
    r1 = R.from_quat(q1)
    r2 = R.from_quat(q2)
    
    # 计算两者之间的相对旋转
    relative_rotation = r1.inv() * r2
    
    # 获取相对旋转的角度（弧度）
    angle_error = relative_rotation.magnitude()  # 角度误差（弧度）
    
    return angle_error

#计算当前位置与期望位置差的角度
def calculate_theta(panda,left_finger_id,right_finger_id,grasp_endpoint_1,grasp_endpoint_2,object_T_global):
    # 获取左手指的位姿
    left_finger_state = p.getLinkState(panda, left_finger_id)
    left_finger_pos, left_finger_orn = left_finger_state[0], left_finger_state[1]
    #print(f"左手指位置: {left_finger_pos}, 方向: {left_finger_orn}")
    # 获取右手指的位姿
    right_finger_state = p.getLinkState(panda, right_finger_id)
    right_finger_pos, right_finger_orn = right_finger_state[0], right_finger_state[1]
    #两个手指实际的向量
    finger_ori=np.array(left_finger_pos)-np.array(right_finger_pos)
    #计算期望的两个夹取点在世界坐标系下的向量
    left_finger_target=objectpoint_T_global(grasp_endpoint_1,object_T_global)
    right_finger_target=objectpoint_T_global(grasp_endpoint_2,object_T_global)
    expect_finger_ori=np.array(left_finger_target)-np.array(right_finger_target)
    #计算joint7需要旋转的角度
    # 1. 计算两个向量的点积
    dot_product = np.dot(expect_finger_ori, finger_ori)
    # 2. 计算两个向量的模
    A_norm = np.linalg.norm(expect_finger_ori)
    B_norm = np.linalg.norm(finger_ori)
    # 3. 计算夹角 (theta)
    cos_theta = dot_product / (A_norm * B_norm)
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))  # 用 np.clip 防止数值误差
    return theta

#给定三个点，计算这三个点形成的平面的法向量
def calculate_normal_from_points(A,B,C):
    # 给定三个点的坐标
    # 计算向量 AB 和 AC
    AB = B - A
    AC = C - A

    # 计算法向量（AB × AC）
    normal_vector = np.cross(AB, AC)
    # 对结果向量进行标准化处理，得到单位向量
    v_normalized = normal_vector / np.linalg.norm(normal_vector)
    return v_normalized

#由给出的三个点来计算夹爪抓取的方向
def calculate_gripper_ori(A,B,C):

    # 定义点 A, B, C 的坐标 (x, y, z)
    A = np.array(A)  # 点 A 的坐标
    B = np.array(B)  # 点 B 的坐标
    C = np.array(C)  # 点 C 的坐标

    # 计算向量 BA 和 BC
    BA = A - B  # 向量 BA = A - B
    BC = C - B  # 向量 BC = C - B

    # 计算叉积 BA × BC，得到垂直的向量
    v = np.cross(BA, BC)
    vec=np.cross(v,BC)

    # 确保向量 v 指向 A
    # 计算 v 和 BA 的点积，决定是否反向
    if np.dot(vec, BA) > 0:
        vec = -vec  # 如果 v 的方向与 BA 的方向相反，则反向 v

    # 对结果向量进行标准化处理，得到单位向量
    v_normalized = vec / np.linalg.norm(vec)

    return v_normalized

#执行关闭夹爪的动作
def close_finger(panda):

    p.setJointMotorControl2(panda, left_finger_id, p.POSITION_CONTROL, targetPosition=0,force=target_force,targetVelocity=target_velocity)

    p.setJointMotorControl2(panda, right_finger_id, p.POSITION_CONTROL, targetPosition=0,force=target_force,targetVelocity=target_velocity)
    for i in range(720):
        p.stepSimulation()
        #time.sleep(time_step)
    #time.sleep(100)

    return 0

#计算抓取位置并执行动作，计算需要目标位置和抓取靠近的角度两个量
def step(panda,object_T_global,grasp_center,grasp_endpoints,log,opnum,center_of_mass):

    #将所有点映射到世界坐标系下，endpoints_center是两个抓取端点的中心，作为目标抓取点
    endpoints_center=(np.array(grasp_endpoints[0])+np.array(grasp_endpoints[1]))/2
    target_position=objectpoint_T_global(endpoints_center,object_T_global)
    grasp_center=objectpoint_T_global(grasp_center,object_T_global)
    left_finger_target=objectpoint_T_global(grasp_endpoints[0],object_T_global)
    right_finger_target=objectpoint_T_global(grasp_endpoints[1],object_T_global)
    #通过将夹爪坐标系里的两个点映射到世界坐标系来确定抓取方向
    gripper_ori=calculate_gripper_ori(grasp_center,left_finger_target,right_finger_target)
    print(gripper_ori)
    rotation_quaternion=gripper_ori_T_rotation_quaternion(gripper_ori)
    #将物体坐标系中的抓取点映射到世界坐标系中作为目标抓取点
    target_position=target_position+args.depth*np.linalg.norm(gripper_ori)
    #target_position=target_position+0.05*np.linalg.norm(gripper_ori)
    #print("希望的抓取点：",target_position)
    #添加辅助线，用来表示希望的抓取点形成的向量
    p.addUserDebugLine(left_finger_target,right_finger_target, [0, 1, 0], lineWidth=5)
    #创建一个球体，用来表示目标抓取点
    sphere_1=create_sphere(target_position,0.005,[1,0,0,0.5])
    sphere_3=create_sphere(grasp_center,0.005,[0,0,1,0.5])

    #对末端执行器进行反向运动学计算，让机械臂末端以指定角度到达目标点
    panda_joint_poses = p.calculateInverseKinematics(
        panda,
        end_effector_id,
        target_position,
        targetOrientation=rotation_quaternion,
        lowerLimits=ll,
        upperLimits=ul,
        jointRanges=jr,
        restPoses=rp,
        maxNumIterations=500
    )
    #根据上面算出来的机械臂关节位置执行动作
    for joint_index in range(len(panda_joint_poses)):
        p.setJointMotorControl2(
            bodyUniqueId=panda,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=panda_joint_poses[joint_index],
            force=target_force  # 增加力矩以抵消重力
    )
    #设置两个夹爪宽度
    p.setJointMotorControl2(panda, left_finger_id, p.POSITION_CONTROL, targetPosition=finger_max_position,
                            force=target_force, maxVelocity=target_velocity)
    p.setJointMotorControl2(panda, right_finger_id, p.POSITION_CONTROL, targetPosition=finger_max_position,
                            force=target_force, maxVelocity=target_velocity)
    #执行仿真，让机械臂到达指定位置
    for j in range(240):  
        p.stepSimulation()
        #time.sleep(time_step)
    
    #判断仿真后得到的机械臂的末端执行器的位置与希望的抓取点的位置的差值，用来判断机械臂位置是否正确
    end_effort_pos = p.getLinkState(panda, end_effector_id)[0]
    end_effector_orn=p.getLinkState(panda, end_effector_id)[1]
    pos_error=np.array(end_effort_pos)-np.array(target_position)
    #如果误差过大，则返回flags=-1,改变机械臂的摆放位置再次仿真
    if np.linalg.norm(pos_error)>0.001 :
        p.removeBody(sphere_1)
        p.removeBody(sphere_3)
        flags=-1
        return flags,panda,-1,log
    orn_error=quaternion_error(end_effector_orn,rotation_quaternion)
    if np.linalg.norm(orn_error)>0.1 :
        p.removeBody(sphere_1)
        p.removeBody(sphere_3)
        flags=-1
        return flags,panda,-1,log

    #获取当前joint6的关节状态，第一个代表旋转角度
    theta_1=p.getJointState(panda,gripper_orn_revolute_id)[0]
    #计算期望抓取方向与现在状态的相差的角度值
    theta=calculate_theta(panda,left_finger_id,right_finger_id,grasp_endpoints[0],grasp_endpoints[1],object_T_global)

    # 控制关节旋转到目标角度
    p.setJointMotorControl2(
        bodyUniqueId=panda,              # Panda 机械臂的 ID
        jointIndex=gripper_orn_revolute_id,                    # panda_joint7 的索引
        controlMode=p.POSITION_CONTROL,  # 使用位置控制模式
        targetPosition=theta_1-theta,    # 设置目标角度
        force=target_force               # 最大控制力
        )
    #执行仿真，让机械臂夹爪旋转到目标位置
    for j in range(240):
        p.stepSimulation()
        #time.sleep(time_step)
        
    #打印执行后的信息，用来判断执行后姿势的准确性
    theta_error=print_message(panda,object_T_global,grasp_endpoints,target_position)
    #如果转动角度不对，则向另一个方向旋转
    if np.linalg.norm(theta_error)>0.1:
        p.setJointMotorControl2(
            bodyUniqueId=panda,             # Panda 机械臂的 ID
            jointIndex=gripper_orn_revolute_id,                   # panda_joint7 的索引
            controlMode=p.POSITION_CONTROL, # 使用位置控制模式
            targetPosition=theta_1+theta,   # 设置目标角度
            force=target_force              # 最大控制力
            )
        for j in range(240):
            p.stepSimulation()
            #time.sleep(time_step)
    theta_error=print_message(panda,object_T_global,grasp_endpoints,target_position)
    if np.linalg.norm(theta_error)>0.1:
        p.removeBody(sphere_1)
        p.removeBody(sphere_3)
        flags=-1
        return flags,panda,-1,log
    #获取左手指关节的位置
    left_finger_pos = p.getLinkState(panda, left_finger_id)[0]
    # 获取右手指关节的位置
    right_finger_pos = p.getLinkState(panda, right_finger_id)[0]
    #添加辅助线，用来表示实际的抓取点形成的向量
    p.addUserDebugLine(left_finger_pos,right_finger_pos, [0, 0, 1], lineWidth=5)
    #重新加载物体，可视化质心位置
    object,constraint = create_object()
    object_pos,object_ori=p.getBasePositionAndOrientation(object)
    sphere_2=create_sphere(object_pos,0.005,[1,1,0])
    #关闭夹爪
    close_finger(panda)
    p.removeBody(sphere_1)
    p.removeBody(sphere_2)
    p.removeBody(sphere_3)
    p.removeAllUserDebugItems()
    #使用相机记录此时的姿势
    carmer_photo(opnum,grasp_center,left_finger_target,right_finger_target,target_position)
    #记录初始时末端执行器的位置和方向，将方向转化为3x3数组
    end_effector_pos_1 = p.getLinkState(panda, end_effector_id)[0]
    object_pos_1,object_ori=p.getBasePositionAndOrientation(object)
    end_effector_orn_1 = p.getLinkState(panda, end_effector_id)[1]
    matrix_1=quaternion_to_rotation_matrix_panda(end_effector_orn_1)
    #记录初始时的机械臂和物体接触信息
    contact_point_1,log=record(panda,object,log,center_of_mass) 
    #撤去对物体的位置限制
    p.removeConstraint(constraint)
    #进行仿真，五秒，认为物体可以保持稳定
    for i in range(1200):
        p.stepSimulation()
        #time.sleep(time_step)
    #记录稳定后末端执行器的位置和方向，将方向转化为3x3数组
    end_effector_pos_2 = p.getLinkState(panda, end_effector_id)[0]
    object_pos_2,object_ori=p.getBasePositionAndOrientation(object)
    end_effector_move_distance = np.linalg.norm(
        np.array(end_effector_pos_1)-np.array(object_pos_1) - np.array(end_effector_pos_2)+np.array(object_pos_2)) 
    end_effector_orn_2 = p.getLinkState(panda, end_effector_id)[1]
    matrix_2 = quaternion_to_rotation_matrix_panda(end_effector_orn_2)
    #计算稳定前后末端执行器移动的空间角（弧度值）
    angle=rotation_matrix_angle(matrix_1,matrix_2)
    #记录稳定时的机械臂和物体接触信息,返回当前接触点
    contact_point_2,log=record(panda,object,log,center_of_mass)
    #如果两个接触点均非0，说明最后抓取成功，返回一个标志值flags用来判断是否继续执行质量增加的仿真
    #if isinstance(contact_point_1, list) and isinstance(contact_point_2, list):
    if len(contact_point_1)>0 and len(contact_point_2)>0:
            flags=True
            if flags==True:
                log.iloc[-1, 2] = angle
                log.iloc[-1, 3] = end_effector_move_distance
                log.at[log.index[-1], '末端执行器位置'].append(tuple(np.array(end_effector_pos_1)-np.array(object_pos_1)))
                log.at[log.index[-1], '末端执行器位置'].append(tuple(np.array(end_effector_pos_2)-np.array(object_pos_2)))  
            return flags,panda,object,log
    else:
        flags=False
        return flags,panda,object,log 

#计算mesh的质心
def get_center_of_mass_from_mesh():
    # 加载mesh
    mesh = trimesh.load_mesh(mesh_path)
    # 获取网格的顶点和面片
    vertices = torch.tensor(mesh.vertices, dtype=torch.float32)  # (N, 3)
    faces = torch.tensor(mesh.faces, dtype=torch.long) 
    center_of_mass,total_area=compute_mesh_center_of_mass_and_area(vertices,faces)
    return center_of_mass,total_area

#计算mesh的质心位置,在物体坐标系中的位置
def compute_mesh_center_of_mass_and_area(vertices: torch.Tensor, faces: torch.Tensor):
    """
    计算网格的质心，使用面片面积加权计算每个三角形的质心。

    参数:
    vertices (torch.Tensor): 顶点坐标 (N, 3)，每个顶点是一个三维坐标。
    faces (torch.Tensor): 面片 (F, 3)，每个面片由三个顶点组成。

    返回:
    center_of_mass (torch.Tensor): 网格质心坐标 (3,)。
    total_area (float): 网格总面积。
    """
    v0 = vertices[faces[:, 0], :]  # (F, 3)
    v1 = vertices[faces[:, 1], :]  # (F, 3)
    v2 = vertices[faces[:, 2], :]  # (F, 3)

    # 计算 (v1 - v0) 和 (v2 - v0)
    v1_minus_v0 = v1 - v0  # (F, 3)
    v2_minus_v0 = v2 - v0  # (F, 3)

    # 计算叉积 (v1 - v0) × (v2 - v0)，得到三角形的法向量
    cross_product = torch.cross(v1_minus_v0, v2_minus_v0, dim=-1)  # (F, 3)

    # 计算三角形的面积，即叉积的模长除以2
    area = torch.norm(cross_product, dim=-1) / 2.0  # (F,)

    # 计算每个三角形的质心
    triangle_center_of_mass = (v0 + v1 + v2) / 3.0  # (F, 3)

    # 计算总的面积和加权质心
    total_area = torch.sum(area)  # 总面积
    weighted_center_of_mass = torch.sum(
        triangle_center_of_mass * area.view(-1, 1), dim=0)  # 加权质心

    # 计算质心
    center_of_mass = weighted_center_of_mass / (total_area + 1e-16)

    return center_of_mass, total_area

#在稳定后逐渐增加物体的质量
def increase_mass(flags,panda,object):
    if flags==True:
        new_mass=1
        for i in range(10000):
        # 每1秒质量增加0.1千克
            new_mass += 0.1
            p.changeDynamics(object, -1, mass=new_mass)
            print(new_mass)
            for i in range(240):
                p.stepSimulation()
                #time.sleep(time_step)
            contact_points = p.getContactPoints(panda, object)  # 获取机械臂与积木的接触点
            if len(contact_points)==0:
                return new_mass

    else :
        return -1
    
# 获得接触点的信息
def get_grasp_contact_points(panda_id, object_id):

    contact_points = p.getContactPoints(panda_id, object_id)  # 获取机械臂与积木的接触点
    contact_info = []

    for contact in contact_points:
        contact_point = {
            'position_gripper': contact[5],  # 接触点在夹爪的位置
            'position_object': contact[6],
            'normal': contact[7],       # 接触点法线
            'distance': contact[8],     # 接触距离（接触物体间的距离，负值表示碰撞）
            'normalforce': contact[9],   # 正向接触力
            'lateralforce1': contact[10],  # 侧向摩擦力
            'lateralforce1_gravity_ori': contact[11],  # 侧向摩擦力方向
            'lateralforce2': contact[12],  # 侧向摩擦力
            'lateralforce2_gravity_ori': contact[13],  # 侧向摩擦力方向
            'contactflag': contact[0],   # 接触类型
            'a': contact[1],       # 第一个物体的链接ID
            'b': contact[2],       # 第二个物体的链接ID
        }
        contact_info.append(contact_point)

    return contact_info

#验证给定的矩阵是否为有效的旋转矩阵。
def is_rotation_matrix(matrix):
    """
    验证给定的矩阵是否为有效的旋转矩阵。
    """
    if matrix.shape != (3, 3):
        return False
    should_be_identity = np.dot(matrix.T, matrix)
    I = np.identity(3, dtype=matrix.dtype)
    return np.allclose(should_be_identity, I) and np.isclose(np.linalg.det(matrix), 1.0)

#计算两个旋转矩阵 R1 和 R2 之间的旋转角度（弧度）
def rotation_matrix_angle(R1, R2):
    """
    计算两个旋转矩阵 R1 和 R2 之间的旋转角度（弧度）。

    参数:
    - R1: 第一个 3x3 旋转矩阵。
    - R2: 第二个 3x3 旋转矩阵。

    返回:
    - 两个旋转矩阵之间的夹角（度）。
    """
    # 验证输入是否为有效的旋转矩阵
    if not is_rotation_matrix(R1):
        raise ValueError("R1 不是一个有效的旋转矩阵。")
    if not is_rotation_matrix(R2):
        raise ValueError("R2 不是一个有效的旋转矩阵。")
    
    # 计算相对旋转矩阵
    R_diff = np.dot(R1.T, R2)
    
    # 使用 scipy 提取旋转角度
    rotation = R.from_matrix(R_diff)
    angle_rad = rotation.magnitude()
    
    # 转换为度
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg

#计算两个点之间的距离和方向
def calculate_distance_and_gravity_ori_world(object_pos, link_pos):
    """计算距离和方向"""
    #距离计算
    distance = np.linalg.norm(
        np.array(object_pos) - np.array(link_pos))  
    gravity_ori = (np.array(object_pos) - np.array(link_pos))  # 在世界坐标系下的方向
    return distance, gravity_ori

#将四元数转换为旋转矩阵
def quaternion_to_rotation_matrix_panda(quat):
    rotation = p.getMatrixFromQuaternion(quat)
    matrix = np.array(rotation).reshape(3, 3)
    return  matrix

#初始化.csv文件
def initialization_log(log):
    log.at[log.index[-1], '末端执行器位置'] = []
    log.at[log.index[-1], '机械臂位置'] = []
    log.at[log.index[-1], '物体位置'] = []
    log.at[log.index[-1], '接触点位置'] = []
    log.at[log.index[-1], '接触点位置'] = []
    log.at[log.index[-1], '法线'] = []
    log.at[log.index[-1], '距离'] = []
    log.at[log.index[-1], '正向接触力'] = []
    log.at[log.index[-1], '侧向摩擦力'] = []
    log.at[log.index[-1], '相对于物体质心的距离'] = []
    log.at[log.index[-1], '在物体坐标系中的坐标'] = []
    return log

#记录接触点信息
def record(panda_id, object,log,center_of_mass):
    # 获取接触点信息
    contact_points = get_grasp_contact_points(panda_id, object)

    if len(contact_points) > 0:
        # 获取物体质心的初始位置和方向
        object_pos, object_orn = p.getBasePositionAndOrientation(object)
        #减去质心的偏移所得到的是物体实际摆放的位置
        #object_pos=np.array(object_pos)-np.array(center_of_mass)
        object_T_global=get_object_matrix(object,center_of_mass)

        # 将接触点信息存储到文件，进行进一步处理

        log.at[log.index[-1],'物体质心位置'] = object_pos
        for contact in contact_points:
                log.at[log.index[-1], '接触点位置'].append(contact['position_gripper'])
                log.at[log.index[-1], '法线'].append(contact['normal'])
                log.at[log.index[-1], '距离'].append(contact['distance'])
                log.at[log.index[-1], '正向接触力'].append(contact['normalforce'])
                log.at[log.index[-1], '侧向摩擦力'].append(contact['lateralforce1'])
                #计算接触点相对于质心的距离
                dictance_contact,gravity_ori_contact=calculate_distance_and_gravity_ori_world(contact['position_gripper'],object_pos)
                log.at[log.index[-1], '相对于物体质心的距离'].append(dictance_contact)
                #计算接触点在物体坐标系的位置
                contact_position=objectpoint_T_global(contact['position_gripper'],np.linalg.inv(object_T_global))
                log.at[log.index[-1], '在物体坐标系中的坐标'].append(tuple(contact_position))

        return contact_position,log

    else :
        log.iloc[log.index[-1],0]='抓取失败'
        empty_list=[] 
        return empty_list,log  

#读取json文件中的信息，返回抓取姿势(从夹爪mesh到物体mesh的变换矩阵，4x4)，抓取中心点(物体坐标系中)，两个抓取端点(物体坐标系中)
def read_json(obj_key,i):
    obj_key=int(obj_key)
    # 读取 JSON 文件并提取 gripper_pose 字段
    with open(json_path, 'r') as f:
        gripper_data = json.load(f)
    
    # 确保 i 不超过 gripper_data 的长度
    if i < len(gripper_data):
        if obj_key < len(gripper_data):
            # 遍历每个 gripper pose
            data = gripper_data[obj_key]
            #gripper_pose = data["gripper_pose"]
            grasp_center = data["center_"]
            grasp_endpoints = data["endpoints"]
            gravity = data["gravity"]
            obj_id = data["obj_id"]
            cam_id = data["cam_id"]
            scene_id = data["scene_id"]
            
            if grasp_center and grasp_endpoints:
                flag = True  # 数据有效
            else:
                flag = False  # 数据无效
        else :
            flag = False
            return 0,0,(0,0,-1),flag
    else :
        flag = False
        return 0,0,(0,0,-1),flag

    return grasp_center,grasp_endpoints,gravity,flag

#计算从物体坐标系到世界坐标系的变化矩阵，4*4的，包括旋转和平移
def get_object_matrix(object,center_of_mass):
    object_pos,object_ori=p.getBasePositionAndOrientation(object)
    #需要减去质心的偏移（因为上面获取的物体位置实际上是质心的位置）
    if isinstance(center_of_mass, torch.Tensor):
        center_of_mass = center_of_mass.cpu().numpy() if center_of_mass.is_cuda else center_of_mass.numpy()
    object_pos=np.array(object_pos)-np.array(center_of_mass)
    # 将四元数转换为旋转矩阵
    rotation_matrix = p.getMatrixFromQuaternion(object_ori)
    # 输出旋转矩阵（返回的旋转矩阵是一个列优先的数组，转换成3x3矩阵）
    rotation_matrix_object_to_world = np.array(rotation_matrix).reshape(3, 3)
    # 获取物体相对于世界坐标系的平移向量，也就是物体的原点位置
    translation_vector_object_to_world = np.array(object_pos)
    #将这两个拼在一起，先构造一个4*4的单位阵，然后写入旋转和平移
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix_object_to_world  
    transformation_matrix[:3, 3] = translation_vector_object_to_world    
    return transformation_matrix

#对向量进行归一化处理，方便计算差值
def normalized(A):
    #计算模大小
    A_norm = np.linalg.norm(A)
    # 归一化
    A_normalized = A / A_norm
    return A_normalized

#用于打印一些机械臂和物体信息，返回当前机器手指的向量与期望向量的差值
def print_message(panda,object_T_global,grasp_endpoints,target_position):
    #获取末端执行器的位姿
    end_effector_state = p.getLinkState(panda, end_effector_id)
    end_effector_pos, end_effector_orn = end_effector_state[0], end_effector_state[1]
    #print(f"末端执行器位置: {end_effector_pos}, 方向: {end_effector_orn}")
    #print("期望末端执行器位置的差",np.array(target_position)-np.array(end_effector_pos))
    # 获取左手指的位姿
    left_finger_state = p.getLinkState(panda, left_finger_id)
    left_finger_pos, left_finger_orn = left_finger_state[0], left_finger_state[1]

    # 获取右手指的位姿
    right_finger_state = p.getLinkState(panda, right_finger_id)
    right_finger_pos, right_finger_orn = right_finger_state[0], right_finger_state[1]

    #计算两个手指实际形成的向量
    finger_ori=np.array(left_finger_pos)-np.array(right_finger_pos)
    #进行归一化
    finger_ori=normalized(finger_ori)

    #计算期望的两个夹取点在世界坐标系下的向量
    left_finger_target=objectpoint_T_global(grasp_endpoints[0],object_T_global)
    right_finger_target=objectpoint_T_global(grasp_endpoints[1],object_T_global)
    expect_finger_ori=np.array(left_finger_target)-np.array(right_finger_target)
    expect_finger_ori=normalized(expect_finger_ori)

    #print("希望的抓取向量：",expect_finger_ori)
    #print("手指实际方向：", finger_ori)
    #print("抓取方向的差",expect_finger_ori-finger_ori)
    return expect_finger_ori-finger_ori

#以（0.5,0.5）为圆心，均匀生成40个点坐标作为机械臂备选位置点
def generate_circle_points(center, radius, num_points):
    # 角度从 0 到 2*pi 均匀分布，顺时针方向
    num_points=int(num_points/4)
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    points_1 = [(center[0] - radius * np.cos(angle), center[1] - radius * np.sin(angle),0.5) for angle in angles]
    points_2 = [(center[0] - radius * np.cos(angle), 0.5,center[2] - radius * np.sin(angle)) for angle in angles]
    points_with_z_03 = [(x,y,z-0.1) for x, y, z in points_1]
    points_with_z_07 = [(x,y,z+0.1) for x, y, z in points_1]
    points=[center]+points_1+points_2+points_with_z_03+points_with_z_07
    return points

#设置重力
def gravity_setting(gravity):
    gravity_ori = np.array(gravity)
    gravity_ori = gravity_ori / np.linalg.norm(gravity_ori)
    gravity = np.array(gravity_ori * 9.8)
    p.setGravity(*gravity)

if __name__=="__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--modle",help="被抓物体obj文件路径", default="obj/scene_0102/9_simple.obj")
    parser.add_argument("-g", "--grasp_json",help="grasp.json文件路径", default="json/scene_0102/0000_0009.json")
    parser.add_argument("-op", "--output_path",help="输出结果文件路径", default="output/scene_0102/09")
    parser.add_argument("-p", "--position", help="机械臂设定的位置", default=[0,0,0])
    parser.add_argument("--center_of_mass", help="物体设定的质心位置,相对物体坐标系而言", default=[0,0,0])
    parser.add_argument("--object_position", help="物体在世界坐标系下的位置", default=[0.5,0.5,0.5])
    parser.add_argument("--num_points",help="每次仿真最多的机械臂采样位置个数",default=100)
    parser.add_argument("-gr","--gravity_ori",help="重力方向",default=(0,0,-1))
    parser.add_argument("-dp","--depth",help="抓取深度",default=0)
    args = parser.parse_args()

    time_step = 1./240.
    total_time = 0
    #物体质量
    mass = 1
    p.connect(p.DIRECT)
    #p.connect(p.GUI)
    # 将一个目录路径添加到 PyBullet 的文件搜索路径中，之后PyBullet 在加载资源文件时，会同时搜索该路径下的文件
    path=p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # 物体设置
    mesh_path = args.modle

    center_of_mass=args.center_of_mass
    object_position = args.object_position
    output_path=args.output_path
    #物体缩放比例
    mesh_scale = [1, 1, 1]
    #机械臂在世界坐标系下的的放置位置，根关节
    gripper_basePosition = args.position

    #夹爪在自己坐标系中的朝向，通过两个点组成的向量表示
    start_point_1 = [1, 0, 0]
    start_point_2 = [0, 0, 0]

    # 定义夹爪初始宽度
    finger_max_position = 0.1  # 最大夹爪位置（初始完全张开）
    #机械臂运动最大速度和力
    target_velocity = 0.2
    target_force = 500
    #物体和夹爪的滑动摩擦系数
    friction_object = 0.6
    friction_gripper = 0.6
    #左右手指，末端执行器索引
    left_finger_id = 7
    right_finger_id = 8
    end_effector_id = 9
    #控制夹爪旋转关节索引
    gripper_orn_revolute_id = 5

    pandaNumDofs = 7
    # 关节的最小角度限制（下限），即每个关节的最小允许位置（弧度）
    ll = [-4]*pandaNumDofs
    # 关节的最大角度限制（上限），即每个关节的最大允许位置（弧度）
    ul = [4]*pandaNumDofs
    # 关节的运动范围（上限 - 下限）,这里为什么设置的不是这样？
    jr = [8]*pandaNumDofs
    jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
    rp = jointPositions
    #设置机械臂采样点
    center=object_position
    # radius = np.linalg.norm(np.array(center) - np.array(gripper_basePosition[:2]))
    num_points = args.num_points
    gripper_pos_list = generate_circle_points(center, 0.05, num_points)

    start_time=time.time()
    base_dir = "filter_obj_json2025.1.13"
    json_base = "json"
    obj_base = "change_obj/FC"
    output_base = "output/increase_mass"
    json_dir = os.path.join(base_dir, json_base)
    obj_dir = os.path.join(base_dir, obj_base)
    output_dir = os.path.join(base_dir, output_base ,obj_base)
    os.makedirs(output_dir,exist_ok=True)
    # 遍历 JSON 文件夹的子文件夹
    for scene_folder in sorted(os.listdir(json_dir)):
            if scene_folder.endswith(".json"):
                # JSON 文件路径
                json_file_path = os.path.join(json_dir, scene_folder)
                # 提取 JSON 文件名中的关键字（不含扩展名）
                json_key = scene_folder.split('_')[-1].split('.')[0]

                obj_file_path=os.path.join(obj_dir,json_key)
                if not os.path.exists(obj_file_path):
                    print(f"警告：未找到对应的 OBJ 文件夹 {obj_file_path}")
                    continue 


                # 找到对应的 OBJ 文件
                matched_obj_file = None
                for obj_file in os.listdir(obj_file_path):
                    if obj_file.endswith(".obj") and obj_file.startswith(f"{int(json_key)}_"):
                        matched_obj_file = obj_file
                        obj_key = obj_file.split('_')[-1].split('.')[0]

                        # 如果找不到对应的 OBJ 文件，跳过
                        if not matched_obj_file:
                            print(f"Warning: No matching OBJ file for {scene_folder}")
                            continue

                        # 读取 OBJ 文件内容
                        obj_file_path = os.path.join(obj_dir,json_key, matched_obj_file)

                        # 将处理后的数据写入到对应的输出文件夹
                        output_scene_subfolder = os.path.join(output_dir, json_key)  # 按数字保存
                        os.makedirs(output_scene_subfolder, exist_ok=True)
                        output_file_path = os.path.join(output_scene_subfolder, f"{json_key}")
                        
                        mesh_path=obj_file_path
                        json_path=json_file_path
                        output_path=output_file_path
                        center_of_mass,total_area=get_center_of_mass_from_mesh()

                        print(f"Processed: {json_file_path},{obj_file_path} -> {output_file_path}")
                        run()
    current_time = time.time() - start_time
    print(f"current_time:{current_time}")
    p.disconnect()

