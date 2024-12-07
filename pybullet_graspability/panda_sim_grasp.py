import time
import numpy as np
import math 
import pybullet as p


useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

mass = 0.5
force = 10
friction_object = 0.8
friction_gripper = 0.8

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions

class PandaSim(object):
  def __init__(self, bullet_client, offset):
    self.bullet_client = bullet_client
    self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
    self.offset = np.array(offset)
    
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    self.legos=[]
    
    self.bullet_client.loadURDF("tray/traybox.urdf", [0+offset[0], 0+offset[1], -0.6+offset[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)
    self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.1, 0.2, -0.5])+self.offset, flags=flags))
    self.bullet_client.changeVisualShape(self.legos[0],-1,rgbaColor=[1,0,0,1])
    self.bullet_client.changeDynamics(self.legos[0], -1, mass=mass)
    self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([ 0, 0.2, -0.6])+self.offset, flags=flags))
    self.bullet_client.changeVisualShape(self.legos[1],-1,rgbaColor=[1,1,0,1])
    self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.2, 0.2, -0.7])+self.offset, flags=flags))
    self.bullet_client.changeVisualShape(self.legos[2],-1,rgbaColor=[1,0,1,1])
    #self.sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.2, 0.2, -0.6])+self.offset, flags=flags)
    #self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.2, 0.2, -0.5])+self.offset, flags=flags)

    # 加载物体的mesh文件 (.obj格式)
    mesh_path = "cylinder.obj"
    #mesh_path = "mesh_opt_mesh_iter_355.obj"
    
    mesh_scale = [ 4, 4, 4]  # 设置合适的缩放比例

    # 创建一个可视化形状，使用 Mesh 文件加载物体
    self.visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                   fileName=mesh_path,
                                   rgbaColor=[1, 1, 1, 1],  # 设置颜色
                                   meshScale=mesh_scale)

    # 创建碰撞形状（通常需要为网格创建近似的碰撞形状）
    self.collision_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                         fileName=mesh_path,
                                         meshScale=mesh_scale)

    # 创建物体并设置初始位置和姿态
    self.body_id = p.createMultiBody(baseMass=mass,  # 设置质量
                            baseCollisionShapeIndex=self.collision_shape,
                            baseVisualShapeIndex=self.visual_shape,
                            basePosition=[-0.1, 0.2, -0.6])  # 物体位置
    # 设置物体的摩擦系数
    self.bullet_client.changeDynamics(self.body_id, linkIndex=-1, lateralFriction=friction_object)  # 修改物体的摩擦系数


    # self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, 0.3, -0.7])+self.offset, flags=flags)
    # self.cubeId = self.bullet_client.loadURDF("E:/research/dataset/grasp/EGAD/test_1/cube1.urdf",np.array( [0, 0.1, -0.7])+self.offset, flags=flags)
    orn=[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
    self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
    index = 0
    self.state = 0
    self.control_dt = 1./240.
    self.finger_target = 0
    self.gripper_height = 0.2
    #create a constraint to keep the fingers centered
    c = self.bullet_client.createConstraint(self.panda,
                       9,
                       self.panda,
                       10,
                       jointType=self.bullet_client.JOINT_GEAR,
                       jointAxis=[1, 0, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
    self.bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
    self.bullet_client.changeDynamics(self.panda, linkIndex=-1, lateralFriction=friction_gripper)  # 修改物体的摩擦系数
 
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
      #print("info=",info)
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1

      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
    self.t = 0.



  def reset(self):
    pass

  
  def update_state(self):
    keys = self.bullet_client.getKeyboardEvents()
    if len(keys)>0:
      for k,v in keys.items():
        if v&self.bullet_client.KEY_WAS_TRIGGERED:
          if (k==ord('0')):
            self.state = 0
          if (k==ord('1')):
            self.state = 1
          if (k==ord('2')):
            self.state = 2
          if (k==ord('3')): # 物体上方 预抓取位置
            self.state = 3
          if (k==ord('4')): # 物体抓取位置
            self.state = 4
          if (k==ord('5')): # 机械手张开
                self.state = 5
          if (k==ord('6')): # 机械手闭合
                self.state = 6
          if (k==ord('7')): # 机械手闭合
                self.state = 7
        if v&self.bullet_client.KEY_WAS_RELEASED:
            self.state = 0
    
  def get_grasp_contact_points(self):
    contact_points = self.bullet_client.getContactPoints(self.panda, self.body_id)  # 获取机械臂与积木的接触点
    contact_info = []
    """ 
contact[0]:contactFlags: 一个整数，表示接触的类型（如是否为新接触），包括 0(接触开始)、1(接触持续)、2(接触结束）。
contact[1]:bodyUniqueIdA: 物体 A 的唯一 ID。
contact[2]:bodyUniqueIdB: 物体 B 的唯一 ID。
contact[3]:linkIndexA: 物体 A 中接触点所在的链接索引。
contact[4]:linkIndexB: 物体 B 中接触点所在的链接索引。
contact[5]:positionOnA: 物体 A 上接触点的位置（世界坐标系下的 3D 坐标）。
contact[6]:positionOnB: 物体 B 上接触点的位置（世界坐标系下的 3D 坐标）。
contact[7]:normalOnB: 接触点法线（方向向量），它指向物体 B。
contact[8]:contactDistance: 接触点之间的距离，通常为负值，表示物体之间的重叠程度（如果物体没有重叠，这个值会是 0 或正值）。
contact[9]:normalForce: 正向接触力,表示接触点之间的法向力（单位：牛顿）。
contact[10]:lateralFrictionForce: 侧向摩擦力,接触点之间的摩擦力大小（单位：牛顿）。
    """

    for contact in contact_points:
        contact_point = {
            'position': contact[5],     # 接触点的位置
            'normal': contact[7],       # 接触点法线
            'distance': contact[8],     # 接触距离（接触物体间的距离，负值表示碰撞）
            'normalforce':contact[9],   # 正向接触力
            'lateralforce':contact[10], # 侧向摩擦力
            'contactflag':contact[0],   # 接触类型
            'a': contact[1],       # 第一个物体的链接ID
            'b': contact[2],       # 第二个物体的链接ID            
        }
        contact_info.append(contact_point)

    return contact_info

    
  def step(self, graspWidth):
    # 设置抓取器张开宽度
    if self.state==6:
      self.finger_target = 0.01
    if self.state==5:
      self.finger_target = 0.04 

    # 测试用
    # self.finger_target = graspWidth
    # # print('self.finger_target = ', self.finger_target)
    # for i in [9,10]:
    #   self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 10)

    # 获取接触点信息
    contact_points = self.get_grasp_contact_points()

    if len(contact_points) > 0:
        for contact in contact_points:
            print(f"接触点位置: {contact['position']}, 法线: {contact['normal']}, 距离: {contact['distance']},\
                  正向接触力：{contact['normalforce']},侧向摩擦力: {contact['lateralforce']}")
        print("0000000000")
        # 可以将接触点信息存储到文件，或者作为状态的一部分进一步处理
        # 例如，保存到一个 CSV 文件
    with open(f"mass_{mass}_force_{force}", 'a') as f:
        for contact in contact_points:
            f.write(f"接触点位置: {contact['position']}, 法线: {contact['normal']}, 距离: {contact['distance']},\
                  正向接触力：{contact['normalforce']},侧向摩擦力: {contact['lateralforce']}\n")
        f.write("\n")


    self.update_state()
    # print("self.state=",self.state)
    #print("self.finger_target=",self.finger_target)

   

    alpha = 0.9 #0.99
    if self.state==1 or self.state==2 or self.state==3 or self.state==4 or self.state==7:
      #gripper_height = 0.034
      self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.03
      # print('self.gripper_height = ', self.gripper_height)

      if self.state == 2 or self.state == 3 or self.state == 7:
        self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.2
        # print('self.gripper_height = ', self.gripper_height)
      
      t = self.t
      self.t += self.control_dt
      
      pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1]+self.gripper_height, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)] # 圆形位置
      if self.state == 3 or self.state== 4:
        # 获取红色积木的位置和方向
        pos, o = self.bullet_client.getBasePositionAndOrientation(self.body_id)    #sphereId self.legos[0]
        pos = [pos[0], self.gripper_height, pos[2]] # 机械手位置
        self.prev_pos = pos
      if self.state == 7:
        pos = self.prev_pos
        diffX = pos[0] - self.offset[0]
        diffZ = pos[2] - (self.offset[2]-0.6)
        self.prev_pos = [self.prev_pos[0] - diffX*0.1, self.prev_pos[1], self.prev_pos[2]-diffZ*0.1]

      	
      orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])   # 机械手方向
      # 根据目标位置计算关节位置
      jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=20)

      for i in range(pandaNumDofs):
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240,maxVelocity=0.2)

    #target for fingers
    for i in [9,10]:
      self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= force ,maxVelocity=0.2)
    

class PandaSimAuto(PandaSim):
  def __init__(self, bullet_client, offset):
    PandaSim.__init__(self, bullet_client, offset)
    self.state_t = 0
    self.cur_state = 0
    self.states=[0, 3, 5, 4, 6, 3, 7]
    self.state_durations=[3, 1, 1, 1, 1, 1, 1]
  
  def update_state(self):
    self.state_t += self.control_dt
    if self.state_t > self.state_durations[self.cur_state]:
      self.cur_state += 1
      if self.cur_state >= len(self.states):
        self.cur_state = 0
      self.state_t = 0
      self.state=self.states[self.cur_state]
      #print("self.state=",self.state)