import pybullet as p
import numpy as np
import time


# 启动 PyBullet 物理引擎（GUI模式）
p.connect(p.GUI)

# 禁用重力
p.setGravity(0, 0, 0)

# 加载物体的mesh文件 (.obj格式)
mesh_path = "new0-process.obj"
mesh_scale = [10, 10, 10]  # 设置合适的缩放比例

# 创建一个可视化形状，使用 Mesh 文件加载物体
visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                   fileName=mesh_path,
                                   rgbaColor=[1, 1, 1, 1],  # 设置颜色
                                   meshScale=mesh_scale)

# 创建碰撞形状（通常需要为网格创建近似的碰撞形状）
collision_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                         fileName=mesh_path,
                                         meshScale=mesh_scale)

# 创建物体并设置初始位置和姿态
body_id = p.createMultiBody(baseMass=1,  # 设置质量
                            baseCollisionShapeIndex=collision_shape,
                            baseVisualShapeIndex=visual_shape,
                            basePosition=[0, 0, 0])  # 物体位置

# 加载一个机器人的URDF模型（例如Panda机器人）
gripper_id = p.loadURDF("/home/xiaofei/anaconda3/envs/pybullet_graspability/lib/python3.8/site-packages/pybullet_data/franka_panda/panda.urdf", basePosition=[0, 0.3, 0],useFixedBase=True)

# 获取机器人末端执行器（抓取器）对应的链接ID
end_effector_link_index = 11  # 例如Panda机器人末端执行器的链接ID

# 定义目标抓取位置（你可以手动设定，也可以通过物体的位置来决定）
object_position, _ = p.getBasePositionAndOrientation(body_id)
target_position = [object_position[0], object_position[1], object_position[2] + 0.1]  # 在物体上方0.1m的位置抓取


# 将目标位置转换为目标姿态（抓取点的姿态）
target_orientation = p.getQuaternionFromEuler([0, 0, 0])  # 假设水平抓取

# 使用反向动力学计算关节角度（IK）
joint_angles = p.calculateInverseKinematics(gripper_id, end_effector_link_index,
                                             targetPosition=target_position,
                                             targetOrientation=target_orientation)

# 将计算出的关节角度设置到机器人模型中
for i in range(len(joint_angles)):
    p.setJointMotorControl2(gripper_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])

# 假设夹爪是通过关节控制的
# 夹爪的关节通常是机器人末端执行器附近的关节（例如第11个关节）
gripper_joint_index = 9  # 对应夹爪的关节ID，具体ID根据URDF模型而定

# 设置夹爪闭合动作
p.setJointMotorControl2(gripper_id, gripper_joint_index, p.POSITION_CONTROL, targetPosition=0)  # 控制夹爪闭合，0.02表示关闭的目标位置，单位是弧度

# 获取物体的重心
com_pos, _ = p.getBasePositionAndOrientation(body_id)
print(com_pos)

# 获取抓取点和接触信息
contact_points = p.getContactPoints(bodyA=body_id, bodyB=gripper_id)

""" 
contact[0]: 第一个物体的ID(碰撞体1)
contact[1]: 第二个物体的ID(碰撞体2)
contact[2]: 第一个物体的链接ID
contact[3]: 第二个物体的链接ID
contact[4]: 距离（接触深度）
contact[5]: 接触位置(contact positio)
contact[6]: 接触点的旋转四元数
contact[7]: 接触法向量(contact normal)
contact[8]: 接触摩擦
contact[9]: 接触力(contact force)
"""

#接触点位置，法向量，接触力
for contact in contact_points:
    print("Contact Position:", contact[5])
    print("Contact Normal:", contact[7])
    print("Contact Force:", contact[9])

file_path = "contact_info.txt"  # 定义文件路径
with open(file_path, "w") as file:
    if len(contact_points) > 0:
        file.write("抓取成功，物体已被夹取！\n")
        for contact in contact_points:
            file.write(f"Contact Position: {contact[5]}\n")  # 位置
            file.write(f"Contact Normal: {contact[7]}\n")    # 法线
            file.write(f"Contact Force: {contact[9]}\n")     # 力
            file.write("\n")  # 每个接触点之间换行
    else:
        file.write("没有接触到物体。\n")

# 等待一会，确保姿势稳定
time.sleep(2)

# 保持GUI界面打开，直到手动关闭
while True:
    p.stepSimulation()  # 步进仿真
    time.sleep(1./240.)  # 控制仿真更新频率（240 FPS）

# 断开连接
p.disconnect()

