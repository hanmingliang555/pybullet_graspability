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
import random


def run():
    log = pd.DataFrame(columns=['抓取结果', '掉落质量(kg)','空间角变化(弧度)','物体质心移动距离','末端执行器位置','机械臂位置','物体位置', '物体质心位置','接触点位置','法线','距离','正向接触力','侧向摩擦力','相对于物体质心的距离','在物体坐标系中的坐标'])
    object,constraint=create_object()
    #计算从物体坐标系到世界坐标系的变化矩阵，4*4的，包括旋转和平移
    object_T_global=get_object_matrix(object,center_of_mass)
    #移除物体，以防机械臂运动过程中碰撞
    p.removeBody(object)
    flags=1
    #依次读取json文件内容进行仿真
    for i in range(1):
        log_1=pd.DataFrame(columns=['抓取场景和物体','力(N)', '扰动力','质量(kg)','空间角变化(弧度)','物体质心移动距离', '物体质心位置','接触点位置','正向接触力','侧向摩擦力','相对于物体质心的距离','在物体坐标系中的坐标'])
        #读取json文件，获得从夹爪坐标系到物体坐标系的变换矩阵，4x4,包括旋转和平移,flag_1代表着文件内是否还有有效数据
        grasp_center,grasp_endpoints,gripper_ori,gravity,flag_1,obj_id,scene_id=read_json(obj_key,i)
        gravity_setting(np.array(gravity))
        # 初始化一行数据为 None
        log.loc[len(log)] = [None] * len(log.columns)
        #初始化设置log
        log=initialization_log(log)
        if flag_1==True:
                # 调用函数生成新的 URDF
                urdf_output_path=generate_urdf(template_urdf, output_urdf,grasp_endpoints,gripper_ori,obj_key)
                panda=create_gripper(urdf_output_path)
                #执行仿真,falgs=-1说明机械臂位置不对，flags=False说明该姿势抓不住，flags=True说明可以抓住物体
                flags,panda,object,log,object_pos_1,matrix_1=step(panda,object_T_global,grasp_center,grasp_endpoints,log,obj_key,center_of_mass)
                #逐渐增加扰动，记录最大扰动力
                max_froce=random_perturbation(flags,panda,object,object_pos_1,matrix_1,log_1,obj_id,scene_id,obj_key,center_of_mass)
                print(max_froce)
                #移除所有物体、机械臂、辅助线
                if object==-1:
                    p.removeBody(panda)
                    p.removeAllUserDebugItems()
                else:
                    p.removeBody(panda)
                    p.removeBody(object)
                    p.removeAllUserDebugItems()
                #记录抓取成功时的信息
                if flags==True or flags == False :
                    print(gripper_basePosition)
                    if flags==True:
                        log.iloc[-1, 0] = '抓取成功'
                    log.iloc[-1, 1] = mass
                    log.at[log.index[-1], "机械臂位置"] = gripper_basePosition
                    log.at[log.index[-1],'物体位置'] = object_position
                    log.to_csv(f'{output_path}_{obj_key}.csv', index=True,na_rep='NA')
                log.to_csv(f'{output_path}_{obj_key}.csv', index=True,na_rep='NA')
            #这里的flags已经从寻找位置的for循环跳出，如果falgs=-1,说明没找到合适位置
                if flags==-1:
                    log.iloc[-1, 0] = '未找到抓取位置'
                    flags=1

        else :
            print("数据读取完毕，仿真结束")
            break

# 加载机械臂
def create_gripper(urdf_output_path):
    panda = p.loadURDF(urdf_output_path,
                    basePosition=object_position,useFixedBase=True)
    p.changeDynamics(panda, linkIndex=-1, lateralFriction=friction_gripper)
    p.createConstraint(parentBodyUniqueId=panda,
                    parentLinkIndex=-1,
                    childBodyUniqueId=-1,
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=object_position,
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

def generate_urdf(template_path, output_path,grasp_endpoints,gripper_ori,j):
    """
    根据模板生成新的 URDF 文件，并设置左右手指的初始点和运动方向。

    :param template_path: 模板 URDF 文件路径
    :param output_path: 生成的 URDF 文件保存路径
    :param left_origin: 左手指初始点 [x, y, z]
    :param left_axis: 左手指运动方向 [x, y, z]
    :param right_origin: 右手指初始点 [x, y, z]
    :param right_axis: 右手指运动方向 [x, y, z]
    """
    # 初始方向和目标方向
    initial_direction = np.array([0, 0, 1])
    left_finger=grasp_endpoints[0]
    right_finger=grasp_endpoints[1]
    middle_finger=grasp_endpoints[2]
    left_ori=gripper_ori[0]
    right_ori=gripper_ori[1]
    middle_ori=gripper_ori[2]

    left_rpy = calculate_rpy(initial_direction,left_ori)
    right_rpy=calculate_rpy(initial_direction,right_ori)
    middle_rpy=calculate_rpy(initial_direction,middle_ori)

    left_axis=world_to_parent_axis(left_ori,left_rpy)
    right_axis=world_to_parent_axis(right_ori,right_rpy)
    middle_axis=world_to_parent_axis(middle_ori,middle_rpy)

    with open(template_path, "r") as template_file:
        urdf_content = template_file.read()

    # 替换左手指的初始点和运动方向
    urdf_content = urdf_content.replace(
        '<origin xyz="0 -0.1 0" rpy="0 0 0"/>',
        f'<origin xyz="{left_finger[0]} {left_finger[1]} {left_finger[2]}" rpy="{left_rpy[0]} {left_rpy[1]} {left_rpy[2]}"/>'
    ).replace(
        '<axis xyz="1 0 0"/>',
        f'<axis xyz="{left_axis[0]} {left_axis[1]} {left_axis[2]}"/>'
    )

    # 替换右手指的初始点和运动方向
    urdf_content = urdf_content.replace(
        '<origin xyz="0 0.1 0" rpy="0 0 0"/>',
        f'<origin xyz="{right_finger[0]} {right_finger[1]} {right_finger[2]}" rpy="{right_rpy[0]} {right_rpy[1]} {right_rpy[2]}"/>'
    ).replace(
        '<axis xyz="0 -1 0"/>',
        f'<axis xyz="{right_axis[0]} {right_axis[1]} {right_axis[2]}"/>'
    )

    urdf_content = urdf_content.replace(
        '<origin xyz="0 0 0.1" rpy="0 0 0"/>',
        f'<origin xyz="{middle_finger[0]} {middle_finger[1]} {middle_finger[2]}" rpy="{middle_rpy[0]} {middle_rpy[1]} {middle_rpy[2]}"/>'
    ).replace(
        '<axis xyz="0 0 1"/>',
        f'<axis xyz="{middle_axis[0]} {middle_axis[1]} {middle_axis[2]}"/>'
    )
    
    output_path=os.path.join(output_path,method_key,obj_base,scene_folder.split('.')[0])
    os.makedirs(output_path,exist_ok=True)
    output_path=os.path.join(output_path,f"{j}.urdf")
    # 将修改后的 URDF 写入新的文件
    with open(output_path, "w") as output_file:
        output_file.write(urdf_content)
    print(f"新的 URDF 文件已生成: {output_path}")
    return output_path

def calculate_rpy(A, B):
    # 归一化向量
    A = A / np.linalg.norm(A)
    B = B / np.linalg.norm(B)

    # 计算旋转轴和角度
    axis = np.cross(A, B)
    angle = np.arccos(np.clip(np.dot(A, B), -1.0, 1.0))

    # 如果 A 和 B 平行（包括反向），特殊处理
    if np.linalg.norm(axis) < 1e-6:
        if np.allclose(A, B):
            return [0, 0, 0]  # 无需旋转
        else:
            return [np.pi, 0, 0]  # 方向相反，旋转 180 度

    axis = axis / np.linalg.norm(axis)  # 归一化旋转轴

    # 使用 Rodrigues' 公式生成旋转矩阵
    rotation_vector = axis * angle
    rotation = R.from_rotvec(rotation_vector)

    # 转换为欧拉角（rpy）
    rpy = rotation.as_euler('xyz', degrees=False)
    return rpy

def world_to_parent_axis(world_axis, rpy):
    # 将 rpy 转换为旋转矩阵 (parent -> world)
    rotation = R.from_euler('xyz', rpy, degrees=False)
    R_parent_to_world = rotation.as_matrix()

    # 计算 R_world_to_parent (转置)
    R_world_to_parent = R_parent_to_world.T

    # 转换方向
    world_axis = np.array(world_axis) / np.linalg.norm(world_axis)  # 确保单位化
    parent_axis = np.dot(R_world_to_parent, world_axis)
    return parent_axis

#使用相机进行每个姿势的截图
def carmer_photo(i,grasp_center,left_finger_target,right_finger_target,target_position):

    up_vector = [0, 0, 1]  # 相机的"上"向量

    camera_position_1 = object_position+np.array([0.3,0,0])
    camera_position_2 = object_position+np.array([-0.3,0,0])
    camera_position_3 = object_position+np.array([0,0.3,0])
    camera_position_4 = object_position+np.array([0,-0.3,0])
    # 设置相机参数
    view_matrix_1 = p.computeViewMatrix(camera_position_1, object_position, up_vector)
    view_matrix_2 = p.computeViewMatrix(camera_position_2, object_position, up_vector)
    view_matrix_3 = p.computeViewMatrix(camera_position_3, object_position, up_vector)
    view_matrix_4 = p.computeViewMatrix(camera_position_4, object_position, up_vector)    
    projection_matrix = p.computeProjectionMatrixFOV(fov=45, aspect=1.0, nearVal=0.1, farVal=100.0)

    width = 640
    height = 480
    img_arr_1 = p.getCameraImage(width, height, viewMatrix=view_matrix_1, projectionMatrix=projection_matrix)
    img_arr_2 = p.getCameraImage(width, height, viewMatrix=view_matrix_2, projectionMatrix=projection_matrix)
    img_arr_3 = p.getCameraImage(width, height, viewMatrix=view_matrix_3, projectionMatrix=projection_matrix)
    img_arr_4 = p.getCameraImage(width, height, viewMatrix=view_matrix_4, projectionMatrix=projection_matrix)
        
    # 获取图像数据
    rgb_img_1 = np.array(img_arr_1[2])
    rgb_img_2 = np.array(img_arr_2[2])
    rgb_img_3 = np.array(img_arr_3[2])  
    rgb_img_4 = np.array(img_arr_4[2])  
    # 保存图像
    image_pil_1 = Image.fromarray(rgb_img_1)
    image_pil_2 = Image.fromarray(rgb_img_2)
    image_pil_3 = Image.fromarray(rgb_img_3)
    image_pil_4 = Image.fromarray(rgb_img_4)
    # 创建绘图对象
    draw_1 = ImageDraw.Draw(image_pil_1)
    draw_2 = ImageDraw.Draw(image_pil_2)
    draw_3 = ImageDraw.Draw(image_pil_3)
    draw_4 = ImageDraw.Draw(image_pil_4)
    # 添加文字到左上角
    text_1 = f'{output_path}_{i}_1.png'
    text_2 = f'{output_path}_{i}_2.png' 
    text_3 = f'{output_path}_{i}_3.png'
    text_4 = f'{output_path}_{i}_4.png' 
    text_position = (10, 10)  # 文字位置，左上角 (x, y)
    draw_1.text(text_position, text_1, fill="black")  # 在第一张图片上添加文字
    draw_2.text(text_position, text_2, fill="black")  # 在第二张图片上添加文字
    draw_3.text(text_position, text_3, fill="black") 
    draw_4.text(text_position, text_4, fill="black")  
    image_pil_1.save(f'{output_path}_{i}_1.png')
    image_pil_2.save(f'{output_path}_{i}_2.png')
    image_pil_3.save(f'{output_path}_{i}_3.png')
    image_pil_4.save(f'{output_path}_{i}_4.png')

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

#计算抓取位置并执行动作，计算需要目标位置和抓取靠近的角度两个量
def step(panda,object_T_global,grasp_center,grasp_endpoints,log,opnum,center_of_mass):

    #将所有点映射到世界坐标系下，endpoints_center是两个抓取端点的中心，作为目标抓取点
    endpoints_center=(np.array(grasp_endpoints[0])+np.array(grasp_endpoints[1]))/2
    target_position=objectpoint_T_global(endpoints_center,object_T_global)
    grasp_center=objectpoint_T_global(grasp_center,object_T_global)
    left_finger_target=objectpoint_T_global(grasp_endpoints[0],object_T_global)
    right_finger_target=objectpoint_T_global(grasp_endpoints[1],object_T_global)

    #添加辅助线，用来表示希望的抓取点形成的向量
    p.addUserDebugLine(left_finger_target,right_finger_target, [0, 1, 0], lineWidth=5)
    #创建一个球体，用来表示目标抓取点
    sphere_1=create_sphere(target_position,0.005,[1,0,0,0.5])

    #重新加载物体，可视化质心位置
    object,constraint = create_object()
    object_pos,object_ori=p.getBasePositionAndOrientation(object)
    sphere_2=create_sphere(object_pos,0.005,[1,1,0])

    #设置两个夹爪宽度
    p.setJointMotorControl2(
                bodyUniqueId=panda,
                jointIndex=left_finger_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.2,
                force=target_force,
                maxVelocity=target_velocity
    )
    p.setJointMotorControl2(
                bodyUniqueId=panda,
                jointIndex=right_finger_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.2,
                force=target_force,
                maxVelocity=target_velocity
    )
    p.setJointMotorControl2(
                bodyUniqueId=panda,
                jointIndex=middle_finger_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.2,
                force=target_force,
                maxVelocity=target_velocity
    )
    for i in range(240):
        for h in range(2):
            p.stepSimulation()
        contact_1=p.getContactPoints(panda,object,left_finger_id)
        for contact in contact_1:
            if contact[8]<-0.003:
                    left_joint=p.getJointState(panda,left_finger_id)[0]
                    p.setJointMotorControl2(
                    bodyUniqueId=panda,
                    jointIndex=left_finger_id,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=left_joint,
                    force=target_force,
                    maxVelocity=target_velocity
                )
        contact_2=p.getContactPoints(panda,object,right_finger_id)
        for contact in contact_2: 
            if contact[8]<-0.003:
                    right_joint=p.getJointState(panda,right_finger_id)[0]    
                    p.setJointMotorControl2(
                        bodyUniqueId=panda,
                        jointIndex=right_finger_id,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=right_joint,
                        force=target_force,
                        maxVelocity=target_velocity
            )
        contact_3=p.getContactPoints(panda,object,middle_finger_id)
        for contact in contact_3: 
            if contact[8]<-0.003:
                    middle_joint=p.getJointState(panda,middle_finger_id)[0]
                    p.setJointMotorControl2(
                        bodyUniqueId=panda,
                        jointIndex=middle_finger_id,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=middle_joint,
                        force=target_force,
                        maxVelocity=target_velocity
            )
        for o in range(5):
            p.stepSimulation()


    #执行仿真，让机械臂到达指定位置
    for j in range(240):  
        p.stepSimulation()
        #time.sleep(time_step)
    p.removeBody(sphere_1)
    p.removeBody(sphere_2)
    p.removeAllUserDebugItems()
    #使用相机记录此时的姿势
    carmer_photo(opnum,grasp_center,left_finger_target,right_finger_target,target_position)
    #记录初始时末端执行器的位置和方向，将方向转化为3x3数组
    object_pos_1,object_ori=p.getBasePositionAndOrientation(object)
    matrix_1=quaternion_to_rotation_matrix_panda(object_ori)
    #记录初始时的机械臂和物体接触信息
    contact_point_1,log=record(panda,object,log,center_of_mass,flag=False) 
    #撤去对物体的位置限制
    p.removeConstraint(constraint)
    #进行仿真，五秒，认为物体可以保持稳定
    for i in range(1200):
        p.stepSimulation()
        #time.sleep(time_step)
    #记录稳定后末端执行器的位置和方向，将方向转化为3x3数组

    object_pos_2,object_ori=p.getBasePositionAndOrientation(object)
    obj_center_move_distance = np.linalg.norm(np.array(object_pos_1)-np.array(object_pos_2)) 
    matrix_2 = quaternion_to_rotation_matrix_panda(object_ori)
    #计算稳定前后末端执行器移动的空间角（弧度值）
    angle=rotation_matrix_angle(matrix_1,matrix_2)
    #记录稳定时的机械臂和物体接触信息,返回当前接触点
    contact_point_2,log=record(panda,object,log,center_of_mass,flag=True)
    #如果两个接触点均非0，说明最后抓取成功，返回一个标志值flags用来判断是否继续执行质量增加的仿真
    #if isinstance(contact_point_1, list) and isinstance(contact_point_2, list):
    if len(contact_point_1)>0 and len(contact_point_2)>0:
            flags=True
            if flags==True:
                log.iloc[-1, 2] = angle
                log.iloc[-1, 3] = obj_center_move_distance 
            return flags,panda,object,log,object_pos_1,matrix_1
    else:
        flags=False
        return flags,panda,object,log,object_pos_1,matrix_1 

#计算mesh的质心
def get_center_of_mass_from_mesh():
    # 加载mesh
    mesh = trimesh.load_mesh(mesh_path)
    # 获取网格的顶点和面片
    vertices = torch.tensor(mesh.vertices, dtype=torch.float32)  # (N, 3)
    faces = torch.tensor(mesh.faces, dtype=torch.long) 
    center_of_mass,total_area=compute_mesh_center_of_mass_and_area(vertices,faces)
    return center_of_mass,total_area

#计算mesh的质心位置
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

#添加随机扰动
def random_perturbation(flags,panda,obj,object_pos_1,matrix_1,log_1,obj_id,scene_id,pose_num,center_of_mass):
    force_ori=np.array([[1,1,1],
    [1,-1,-1],
    [-1,1,-1],
    [-1,-1,1]])
    random_force=0
    if flags==True:
        for i in range(10000):
            random_force+=1
            for j in range(4):
                for n in range(240):
                    p.stepSimulation()
                obj_pos,obj_ori=p.getBasePositionAndOrientation(obj)
                # 应用随机力，力作用位置和力大小
                force=random_force*force_ori[j]
                for r in range(2):
                    p.applyExternalForce(objectUniqueId=obj, linkIndex=-1, forceObj=force, posObj=obj_pos, flags=p.WORLD_FRAME)
                    p.stepSimulation()

                # p.setJointMotorControl2(
                #             bodyUniqueId=panda,
                #             jointIndex=left_finger_id,
                #             controlMode=p.POSITION_CONTROL,
                #             targetPosition=0.2,
                #             force=target_force,
                #             maxVelocity=target_velocity
                # )
                # p.setJointMotorControl2(
                #             bodyUniqueId=panda,
                #             jointIndex=right_finger_id,
                #             controlMode=p.POSITION_CONTROL,
                #             targetPosition=0.2,
                #             force=target_force,
                #             maxVelocity=target_velocity
                # )
                # p.setJointMotorControl2(
                #             bodyUniqueId=panda,
                #             jointIndex=middle_finger_id,
                #             controlMode=p.POSITION_CONTROL,
                #             targetPosition=0.2,
                #             force=target_force,
                #             maxVelocity=target_velocity
                # )
                # for i in range(240):
                #     p.stepSimulation()
                #     contact_1=p.getContactPoints(panda,obj,left_finger_id)
                #     for contact in contact_1:
                #         if contact[8]<-0.001:
                #             left_joint=p.getJointState(panda,left_finger_id)[0]
                #             p.setJointMotorControl2(
                #                     bodyUniqueId=panda,
                #                     jointIndex=left_finger_id,
                #                     controlMode=p.POSITION_CONTROL,
                #                     targetPosition=left_joint,
                #                     force=target_force
                #                 )
                #     contact_2=p.getContactPoints(panda,obj,right_finger_id)
                #     for contact in contact_2: 
                #         if contact[8]<-0.001:
                #             right_joint=p.getJointState(panda,right_finger_id)[0]    
                #             p.setJointMotorControl2(
                #                         bodyUniqueId=panda,
                #                         jointIndex=right_finger_id,
                #                         controlMode=p.POSITION_CONTROL,
                #                         targetPosition=right_joint,
                #                         force=target_force
                #             )
                #     contact_3=p.getContactPoints(panda,obj,middle_finger_id)
                #     for contact in contact_3: 
                #         if contact[8]<-0.001:
                #             middle_joint=p.getJointState(panda,middle_finger_id)[0]
                #             p.setJointMotorControl2(
                #                         bodyUniqueId=panda,
                #                         jointIndex=middle_finger_id,
                #                         controlMode=p.POSITION_CONTROL,
                #                         targetPosition=middle_joint,
                #                         force=target_force
                #             )
                #     for o in range(10):  
                #         p.stepSimulation()
            for t in range(240):
                p.stepSimulation()
            print(random_force)
            contact_points = p.getContactPoints(panda, obj)  # 获取机械臂与积木的接触点
            if len(contact_points)==0:
                return random_force
            left_joint_states = p.getJointState(panda, left_finger_id)
            left_actual_force = left_joint_states[3]
            right_joint_states = p.getJointState(panda, right_finger_id)
            right_actual_force = right_joint_states[3]
            middle_joint_states = p.getJointState(panda, middle_finger_id)
            middle_actual_force = middle_joint_states[3]
            print(f"Actual force: {left_actual_force},{right_actual_force},{middle_actual_force}")
            object_pos_2,object_ori=p.getBasePositionAndOrientation(obj)
            obj_center_move_distance = np.linalg.norm(np.array(object_pos_1)-np.array(object_pos_2)) 
            matrix_2 = quaternion_to_rotation_matrix_panda(object_ori)
            #计算稳定前后末端执行器移动的空间角（弧度值）
            angle=rotation_matrix_angle(matrix_1,matrix_2)
            # 初始化一行数据为 None
            log_1.loc[len(log_1)] = [None] * len(log_1.columns)
            #初始化设置log
            log_1=initialization_log_1(log_1)
            log_1.iloc[-1, 0] = f"{scene_id}_{obj_id}_{pose_num}"
            log_1.iloc[-1, 1] = f"left:{left_actual_force},right:{right_actual_force},middle:{middle_actual_force}"
            log_1.iloc[-1, 2] = random_force
            log_1.iloc[-1, 3] = mass
            log_1.iloc[-1, 4] = angle
            log_1.iloc[-1, 5] = obj_center_move_distance
            log_1=increase_random_force(panda,obj,log_1,center_of_mass)
            log_1.to_csv(f'{output_path}_{pose_num}_random_perturbation.csv', index=True,na_rep='NA')
        return random_force

    else :
        return -1

def increase_random_force(panda,object,log_1,center_of_mass):
        # 获取接触点信息
    contact_points = get_grasp_contact_points(panda, object)

    if len(contact_points) > 0:
        # 获取物体质心的初始位置和方向
        object_pos, object_orn = p.getBasePositionAndOrientation(object)
        #减去质心的偏移所得到的是物体实际摆放的位置
        #object_pos=np.array(object_pos)-np.array(center_of_mass)
        object_T_global=get_object_matrix(object,center_of_mass)

        # 将接触点信息存储到文件，进行进一步处理
        log_1.at[log_1.index[-1],'物体质心位置'] = object_pos
        for contact in contact_points:
                log_1.at[log_1.index[-1], '接触点位置'].append(contact['position_gripper'])
                log_1.at[log_1.index[-1], '正向接触力'].append(contact['normalforce'])
                log_1.at[log_1.index[-1], '侧向摩擦力'].append(contact['lateralforce1'])
                #计算接触点相对于质心的距离
                dictance_contact,gravity_ori_contact=calculate_distance_and_gravity_ori_world(contact['position_gripper'],object_pos)
                log_1.at[log_1.index[-1], '相对于物体质心的距离'].append(dictance_contact)
                #计算接触点在物体坐标系的位置
                contact_position=objectpoint_T_global(contact['position_gripper'],np.linalg.inv(object_T_global))
                log_1.at[log_1.index[-1], '在物体坐标系中的坐标'].append(tuple(contact_position))

        return log_1

    else :
        return log_1  

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

#初始化log.csv文件
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

#初始化log_1.csv文件
def initialization_log_1(log_1):
    log_1.at[log_1.index[-1], '接触点位置'] = []
    log_1.at[log_1.index[-1], '正向接触力'] = []
    log_1.at[log_1.index[-1], '侧向摩擦力'] = []
    log_1.at[log_1.index[-1], '相对于物体质心的距离'] = []
    log_1.at[log_1.index[-1], '在物体坐标系中的坐标'] = []
    return log_1

#记录接触点信息
def record(panda_id, object,log,center_of_mass,flag):
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
                if flag==True :
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
            gripper_ori = data["vector"]
            
            if grasp_center and grasp_endpoints:
                flag = True  # 数据有效
            else:
                flag = False  # 数据无效
        else :
            flag = False
            return 0,0,(0,0,-1),flag,0,0
    else :
        flag = False
        return 0,0,0,(0,0,-1),flag,0,0

    # # 提取旋转矩阵和位移向量
    # rotation_matrix = np.array(gripper_pose[:3])  # 旋转矩阵 3x3
    # translation_vector = np.array(gripper_pose[3:])  # 位移向量 3
    # # 创建 4x4 变换矩阵
    # transformation_matrix = np.eye(4)  # 创建 4x4 的单位矩阵
    # transformation_matrix[:3, :3] = rotation_matrix  # 设置旋转矩阵
    # transformation_matrix[:3, 3] = translation_vector  # 设置平移向量

    return grasp_center,grasp_endpoints,gripper_ori,gravity,flag,obj_id,scene_id

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

#设置重力
def gravity_setting(gravity):
    gravity_ori = np.array(gravity)
    gravity_ori = gravity_ori / np.linalg.norm(gravity_ori)
    gravity = np.array(gravity_ori * 9.8)
    p.setGravity(*gravity)

if __name__=="__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--modle",help="被抓物体obj文件路径", default="filter_obj_json/json/0000_0000.json")
    parser.add_argument("-g", "--grasp_json",help="grasp.json文件路径", default="merged_json/0000_0051.json")
    parser.add_argument("-op", "--output_path",help="输出结果文件路径", default="filter_obj_json/output")
    parser.add_argument("-p", "--position", help="机械臂设定的位置", default=[0,0,0])
    parser.add_argument("--center_of_mass", help="物体设定的质心位置,相对物体坐标系而言", default=[0,0,0])
    parser.add_argument("--object_position", help="物体在世界坐标系下的位置", default=[0.5,0.5,0.5])

    parser.add_argument("-gr","--gravity_ori",help="重力方向",default=(0,0,-1))
    parser.add_argument("-dp","--depth",help="抓取深度",default=0)
    args = parser.parse_args()
    random.seed(55)

    time_step = 1./240.
    total_time = 0
    #物体质量
    mass = 1
    p.connect(p.DIRECT)
    #p.connect(p.GUI)
    # 将一个目录路径添加到 PyBullet 的文件搜索路径中，之后PyBullet 在加载资源文件时，会同时搜索该路径下的文件
    path=p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # 物体设置
    #mesh_path = args.modle

    object_position = args.object_position
    #output_path=args.output_path
    template_urdf = "3cylinder.urdf"  # 模板文件路径
    output_urdf = "3cylinder_generatedurdf"   # 输出文件路径
    #物体缩放比例
    mesh_scale = [1, 1, 1]
    #机械臂在世界坐标系下的的放置位置，根关节
    gripper_basePosition = args.position

    #夹爪在自己坐标系中的朝向，通过两个点组成的向量表示
    start_point_1 = [1, 0, 0]
    start_point_2 = [0, 0, 0]

    #机械臂运动最大速度和力
    target_velocity = 0.2
    target_force = 50
    #物体和夹爪的滑动摩擦系数
    friction_object = 0.6
    friction_gripper = 0.6
    #左右手指，末端执行器索引
    left_finger_id = 0
    right_finger_id = 1
    middle_finger_id=2

    start_time=time.time()
    base_dir = "3gripper_data"
    json_base = "json"
    obj_base = "change_obj/FR"
    output_base = "output"
    method_key="random_perturbation"
    json_dir = os.path.join(base_dir, json_base)
    obj_dir = os.path.join(base_dir, obj_base)
    output_dir = os.path.join(base_dir, output_base,method_key,f"{obj_base}_1")
    os.makedirs(output_dir,exist_ok=True)
    start_time=time.time()
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