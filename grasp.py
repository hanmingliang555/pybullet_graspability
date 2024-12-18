import time
import numpy as np
import math
import pybullet as p
import trimesh
import pybullet_data
import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-m", "--modle",help="被抓物体obj文件路径", default="1e5a.obj")
parser.add_argument("-g", "--grasp_json",help="grasp.json文件路径", default="./1_g.json")
parser.add_argument("-p", "--position", help="机械臂设定的位置", default=[0,0,0])
parser.add_argument( "--center_of_mass", help="物体设定的质心位置", default=[0,0,0])
args = parser.parse_args()

time_step = 1/240
total_time = 0
object_created = False
mass = 1

p.connect(p.GUI)
# 设置重力
p.setGravity(0, 0, -9.8)
# 将一个目录路径添加到 PyBullet 的文件搜索路径中，之后PyBullet 在加载资源文件时，会同时搜索该路径下的文件
path=p.setAdditionalSearchPath(pybullet_data.getDataPath())
# 物体设置
mesh_path = args.modle
# 缩放比例
mesh_scale = [1, 1, 1]
#物体在世界坐标系下的初始位置，质心
object_position = [0.5, 0.5, 0.5]
#机械壁在世界坐标系下的的放置位置，根关节
gripper_basePosition = args.position

#夹爪在自己坐标系中的朝向，通过两个点组成的向量表示
start_point_1 = [0, 0, 1]
start_point_2 = [0, 0, 0]


# 定义闭合范围
finger_closing_speed = 0.001  # 每次步进减小的角度
finger_min_position = 0.0  # 最小夹爪位置
finger_max_position = 0.04  # 最大夹爪位置（初始完全张开）
#希望的夹爪宽度，这个有必要设定吗？
target_grasp_width = 0.1
#机械臂运动最大速度
target_velocity = 0.02
target_force = 1000
#物体和夹爪的滑动摩擦系数
friction_object = 1
friction_gripper = 1
#设置物体的质心
center_of_mass=args.center_of_mass
# 左手指是关节 9,右手指是关节 10
left_finger_id = 9
right_finger_id = 10
# 获取 Panda 的末端执行器索引（例如，夹爪中心点）
end_effector_id = 11

pandaNumDofs = 7
# 关节的最小角度限制（下限），即每个关节的最小允许位置（弧度）
ll = [-7]*pandaNumDofs
# 关节的最大角度限制（上限），即每个关节的最大允许位置（弧度）
ul = [7]*pandaNumDofs
# 关节的运动范围（上限 - 下限）,这里为什么设置的不是这样？
jr = [7]*pandaNumDofs
jointPositions = [0, 0, 0, 0, 0, 0, 0, 0, 0]
rp = jointPositions
# 加载一个简单的平面作为地面
#plane_id = p.loadURDF("plane.urdf")  

# 加载机械臂，将它的位置固定在gripper_basePosition
def create_gripper():
    panda = p.loadURDF("franka_panda/panda.urdf",
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
                                       rgbaColor=[1, 1, 1, 0.5],  # 设置颜色
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
                               baseOrientation=[0, 0, 0, 1])
    p.changeDynamics(object, linkIndex=-1, lateralFriction=friction_object)
    # 通过创建约束的方式固定物体在object_position
    constraint=p.createConstraint(parentBodyUniqueId=object,
                   parentLinkIndex=-1,  # 基座固定
                   childBodyUniqueId=-1,  # 没有子物体
                   childLinkIndex=-1,
                   jointType=p.JOINT_FIXED,  # 固定约束
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, 0],
                   childFramePosition=center_of_mass)  # 固定位置在object_position
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
    default_direction = np.array([0, 0, 1])

    # 计算旋转轴（通过叉积获得）
    rotation_axis = np.cross(default_direction, target_ori)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)  # 归一化
    # 计算旋转角度（通过点积获得）
    cos_theta = np.dot(default_direction, target_ori)
    theta = np.arccos(cos_theta)  # 旋转角度
    # 将旋转轴和旋转角度转换为四元数
    # 使用旋转轴和旋转角度来创建四元数
    rotation_quaternion = p.getQuaternionFromAxisAngle(rotation_axis, theta)
    return rotation_quaternion

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

#执行关闭夹爪的动作
def close_finger(panda):

    p.setJointMotorControl2(panda, 9, p.POSITION_CONTROL, targetPosition=0,force=target_force,targetVelocity=0.001)

    p.setJointMotorControl2(panda, 10, p.POSITION_CONTROL, targetPosition=0,force=target_force,targetVelocity=0.1)

    return 0

#计算抓取位置并执行动作，计算需要目标位置和抓取靠近的角度两个量
def step(panda,gripper_T_object,object_T_global,grasp_center,grasp_endpoints):

    #通过将夹爪坐标系里的两个点映射到世界坐标系来确定抓取方向
    gripper_point_1=gripperpoint_T_global(start_point_1,gripper_T_object,object_T_global)
    gripper_point_2=gripperpoint_T_global(start_point_2,gripper_T_object,object_T_global)
    gripper_ori=np.array(gripper_point_1)-np.array(gripper_point_2)
    rotation_quaternion=gripper_ori_T_rotation_quaternion(gripper_ori)
    #将物体坐标系中的抓取点映射到世界坐标系中作为目标抓取点
    target_position=objectpoint_T_global(grasp_center,object_T_global)
    print("希望的抓取点：",target_position)
    #创建一个球体，用来表示希望的抓取中心
    #create_sphere(target_position,0.003,[1,0,0,1])

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
    p.setJointMotorControl2(panda, 9, p.POSITION_CONTROL, targetPosition=finger_max_position,
                            force=target_force, maxVelocity=target_velocity)
    p.setJointMotorControl2(panda, 10, p.POSITION_CONTROL, targetPosition=finger_max_position,
                            force=target_force, maxVelocity=target_velocity)
    #执行仿真，让机械臂到达指定位置
    for j in range(720):  
        p.stepSimulation()
        time.sleep(time_step) 
    #获取当前joint6的关节状态，第一个代表旋转角度
    theta_1=p.getJointState(panda,6)[0]
    #计算期望抓取方向与现在状态的相差的角度值
    theta=calculate_theta(panda,left_finger_id,right_finger_id,grasp_endpoints[0],grasp_endpoints[1],object_T_global)

    # 控制关节旋转到目标角度
    p.setJointMotorControl2(
        bodyUniqueId=panda,          # Panda 机械臂的 ID
        jointIndex=6,      # panda_joint7 的索引
        controlMode=p.POSITION_CONTROL,  # 使用位置控制模式
        targetPosition=theta_1-theta,  # 设置目标角度
        force=target_force     # 最大控制力
        )
    #执行仿真，让机械臂夹爪旋转到目标位置
    for j in range(720):
        p.stepSimulation()
        time.sleep(time_step)
        
    #打印执行后的信息，用来判断执行后姿势的准确性
    print_message(panda,object_T_global,grasp_endpoints,target_position)

    #获取左手指关节的位置
    left_finger_pos = p.getLinkState(panda, left_finger_id)[0]
    # 获取右手指关节的位置
    right_finger_pos = p.getLinkState(panda, right_finger_id)[0]
    #计算期望的两个夹取点在世界坐标系下的向量
    left_finger_target=objectpoint_T_global(grasp_endpoints[0],object_T_global)
    right_finger_target=objectpoint_T_global(grasp_endpoints[1],object_T_global)
    #添加辅助线，用来表示希望的抓取点形成的向量
    p.addUserDebugLine(left_finger_target,right_finger_target, [0, 1, 0], lineWidth=5)
    #添加辅助线，用来表示实际的抓取点形成的向量
    p.addUserDebugLine(left_finger_pos,right_finger_pos, [0, 0, 1], lineWidth=5)

    object,constraint = create_object() 
    time.sleep(5)
    #关闭夹爪
    #close_finger(panda)
    # 运行仿真步
    for i in range(480):
        p.stepSimulation()
        time.sleep(time_step)
    record(panda,object) 
    p.removeConstraint(constraint)
    for i in range(1200):
        p.stepSimulation()
        time.sleep(time_step)
    record(panda,object) 

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
            'lateralforce1_direction': contact[11],  # 侧向摩擦力方向
            'lateralforce2': contact[12],  # 侧向摩擦力
            'lateralforce2_direction': contact[13],  # 侧向摩擦力方向
            'contactflag': contact[0],   # 接触类型
            'a': contact[1],       # 第一个物体的链接ID
            'b': contact[2],       # 第二个物体的链接ID
        }
        contact_info.append(contact_point)

    return contact_info

#计算两个点之间的距离和方向
def calculate_distance_and_direction_world(object_pos, link_pos):
    """计算距离和方向"""
    #距离计算
    distance = np.linalg.norm(
        np.array(object_pos) - np.array(link_pos))  
    direction = (np.array(object_pos) - np.array(link_pos))  # 在世界坐标系下的方向
    return distance, direction

#将四元数转换为旋转矩阵，然后计算相对于机械臂不同位置的方向
def quaternion_to_rotation_matrix_panda(quat, direction):
    """将四元数转换为旋转矩阵，然后计算相对于机械臂不同位置的方向"""
    rotation = p.getMatrixFromQuaternion(quat)
    matrix = np.array(rotation).reshape(3, 3)
    direction_panda = np.dot(matrix.T, direction)
    return direction_panda, matrix

#记录接触点信息
def record(panda_id, object_id):
    # 获取接触点信息
    contact_points = get_grasp_contact_points(panda_id, object_id)

    if len(contact_points) > 0:

        # 获取物体的初始位置和方向
        object_pos, object_orn = p.getBasePositionAndOrientation(object_id)
        object_pos=np.array(object_pos)-np.array(center_of_mass)
        object_T_global=get_object_matrix(object)

        # 获取左手指的位姿
        left_finger_state = p.getLinkState(panda, left_finger_id)
        left_finger_pos, left_finger_orn = left_finger_state[0], left_finger_state[1]

        # 获取右手指的位姿
        right_finger_state = p.getLinkState(panda, right_finger_id)
        right_finger_pos, right_finger_orn = right_finger_state[0], right_finger_state[1]

        # 获取末端执行器的位姿
        end_effector_state = p.getLinkState(panda, end_effector_id)
        end_effector_pos, end_effector_orn = end_effector_state[0], end_effector_state[1]

        # 计算相对于左手指的距离和方向
        distance_left, direction_left = calculate_distance_and_direction_world(
            object_pos, left_finger_pos)
        direction_left, left_rotation = quaternion_to_rotation_matrix_panda(
            left_finger_orn, direction_left)

        # 计算相对于右手指的距离和方向
        distance_right, direction_right = calculate_distance_and_direction_world(
            object_pos, right_finger_pos)
        direction_right, right_rotation = quaternion_to_rotation_matrix_panda(
            right_finger_orn, direction_right)

        # 计算相对于末端执行器的距离和方向
        distance_end, direction_end = calculate_distance_and_direction_world(
            object_pos, end_effector_pos)
        direction_end, end_effector_rotation = quaternion_to_rotation_matrix_panda(
            end_effector_orn, direction_end)

        # 将接触点信息存储到CSV文件，进行进一步处理
        with open(f"{mesh_path}_mass_{mass}_force_{target_force}", 'a') as f:
            for contact in contact_points:
                f.write(f"接触点位置: {contact['position_gripper']}, 法线: {contact['normal']}, 距离: {contact['distance']},\正向接触力：{contact['normalforce']},侧向摩擦力: {contact['lateralforce1']}\n")
                f.write(f"物体质心位置: {object_pos}\n")
                #计算接触点相对于质心的距离
                dictance_contact,direction_contact=calculate_distance_and_direction_world(contact['position_gripper'],object_pos)
                f.write(f"接触点相对于物体质心的距离: {dictance_contact}\n")
                contact_position=objectpoint_T_global(contact['position_gripper'],np.linalg.inv(object_T_global))
                f.write(f"接触点在物体坐标系中的坐标: {contact_position}\n")
            f.write("\n")

            
            # f.write(
            #     f"相对于左手指的距离: {distance_left},方向: {direction_left},矩阵：{left_rotation}\n")
            # f.write(
            #     f"相对于右手指的距离: {distance_right},方向: {direction_right},矩阵：{right_rotation}\n")
            # f.write(
            #     f"相对于末端执行器的距离: {distance_end},方向: {direction_end},矩阵：{end_effector_rotation}\n")
            # f.write("\n")

#读取json文件中的信息，返回抓取姿势(从夹爪mesh到物体mesh的变换矩阵，4x4)，抓取中心点(物体坐标系中)，两个抓取端点(物体坐标系中)
def read_json():
    # 读取 JSON 文件并提取 gripper_pose 字段
    with open(args.grasp_json, 'r') as f:
        gripper_data = json.load(f)

    # 遍历每个 gripper pose
    for i, data in enumerate(gripper_data[1:2]):
        gripper_pose = data["gripper_pose"]
        grasp_center = data["center_"]
        grasp_endpoints = data["endpoints"]

    # 提取旋转矩阵和位移向量
    rotation_matrix = np.array(gripper_pose[:3])  # 旋转矩阵 3x3
    translation_vector = np.array(gripper_pose[3:])  # 位移向量 3
    # 创建 4x4 变换矩阵
    transformation_matrix = np.eye(4)  # 创建 4x4 的单位矩阵
    transformation_matrix[:3, :3] = rotation_matrix  # 设置旋转矩阵
    transformation_matrix[:3, 3] = translation_vector  # 设置平移向量

    return transformation_matrix,grasp_center,grasp_endpoints

#计算从物体坐标系到世界坐标系的变化矩阵，4*4的，包括旋转和平移
def get_object_matrix(object):
    object_pos,object_ori=p.getBasePositionAndOrientation(object)
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

#对象量进行归一化处理
def normalized(A):
    #计算模大小
    A_norm = np.linalg.norm(A)
    # 归一化
    A_normalized = A / A_norm
    return A_normalized

#用于打印一些机械臂和物体信息
def print_message(panda,object_T_global,grasp_endpoints,target_position):
    #获取末端执行器的位姿
    end_effector_state = p.getLinkState(panda, end_effector_id)
    end_effector_pos, end_effector_orn = end_effector_state[0], end_effector_state[1]
    print(f"末端执行器位置: {end_effector_pos}, 方向: {end_effector_orn}")
    print("期望末端执行器位置的差",np.array(target_position)-np.array(end_effector_pos))
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

    print("希望的抓取向量：",expect_finger_ori)
    print("手指实际方向：", finger_ori)
    print("抓取方向的差",expect_finger_ori-finger_ori)
    return 0


panda=create_gripper()
object,constraint=create_object()
time.sleep(5)
#读取json文件，获得从夹爪坐标系到物体坐标系的变换矩阵，4x4,包括旋转和平移
gripper_T_object,grasp_center,grasp_endpoints=read_json()
#计算从物体坐标系到世界坐标系的变化矩阵，4*4的，包括旋转和平移
object_T_global=get_object_matrix(object)
p.removeBody(object)
step(panda,gripper_T_object,object_T_global,grasp_center,grasp_endpoints)

while True:
    p.stepSimulation()
    total_time += time_step
#    record(panda, object)
#    if total_time >= 10:
#        break
    time.sleep(time_step)




