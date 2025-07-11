import pybullet as p
import pybullet_data
import time
import math as m
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R

class RobotSimulation:
    def __init__(self):
        self.Running = True
        self.lock = threading.Lock()

        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)

        # Robot
        self.Robot = p.loadURDF(r"C:\Users\qkrtp.DESKTOP-60UHPRL\PycharmProjects\DoosanRobotics\robot.urdf",useFixedBase=True)
        self.T_EE = None
        self.HomePosition = np.array([0, 0, m.pi / 2, 0, m.pi / 2, 0])
        self.q = self.HomePosition
        self.Send_RobotConfig()
        self.Get_T_EE()

        # Capsule
        self.Capsule = None
        self.Create_Capsule()
        T = np.eye(4)
        T[:3,3] = [0,0,0]/1000
        # World는 Vision에서의 기준좌표계, Pybullet의 기준좌표계는 Observer라 칭함
        self.T_Observer2World = T
        self.P_World2Capsule = np.array([[0],[0],[0]])


    def Send_RobotConfig(self):
        if p.getConnectionInfo()['isConnected']:
            for j in range(6):
                p.resetJointState(self.Robot, j, self.q[j])


    def Get_T_EE(self):
        if p.getConnectionInfo()['isConnected']:
            state = p.getLinkState(self.Robot, linkIndex = 0)
            pos = state[4]
            orn = state[5]
            rot_matrix = p.getMatrixFromQuaternion(orn)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)
            T_EE = np.eye(4)
            T_EE[:3, :3] = rot_matrix
            T_EE[:3, 3] = pos

            self.T_EE = T_EE


    def Simulation(self):
        print("Start Simulation!")
        while self.Running:
            if p.getConnectionInfo()['isConnected']:
                with self.lock:
                    self.Send_RobotConfig()
                    self.Get_T_EE()
                p.stepSimulation()
                time.sleep(0.05)
            else:
                self.Running = False
        print("End Simulation!")


    def Create_Capsule(self):
        vis_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                           fileName="CAD\capsule.stl",
                                           meshScale=[1, 1, 1],
                                           rgbaColor=[1, 0, 0, 0.5])

        self.Capsule = p.createMultiBody(baseMass=1,
                                    baseVisualShapeIndex=vis_shape_id,
                                    basePosition=[0, 0, 0],
                                    baseOrientation=[0, 0, 0, 1])


    def Send_CapsuleConfig(self):
        if p.getConnectionInfo()['isConnected']:

            with self.lock:
                T_Observer2Capsule = self.Get_T_EE()
                T_Observer2Capsule[:3,3] = self.T_Observer2World * np.vstack(self.P_World2Capsule, np.array([1]))
                R_Observer2Capsule = T_Observer2Capsule[:3,:3]
                rot = R.from_matrix(R_Observer2Capsule)
                quat = rot.as_quat()  # [x, y, z, w] 순서
                p.resetJointState(self.Capsule, T_Observer2Capsule[:3,3], quat)

