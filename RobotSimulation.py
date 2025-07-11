import pybullet as p
import pybullet_data
import time
import math as m
import numpy as np

class RobotSimulaition:
    def __init__(self):

        self.Running = True
        self.physicsClient = p.connect(p.GUI)
        self.robotId = p.loadURDF(r"C:\Users\qkrtp.DESKTOP-60UHPRL\PycharmProjects\DoosanRobotics\robot.urdf",useFixedBase=True)
        self.HomePosition = np.array([0, 0, m.pi/2, 0, m.pi/2, 0])
        self.Send_q(self.HomePosition)
        self.T_EE = None
        self.Get_T_EE()


    def Send_q(self, q):
        for j in range(6):
            p.resetJointState(self.robotId, j, q[j])


    def Get_T_EE(self):
        state = p.getLinkState(self.robotId, linkIndex = 0)
        pos = state[4]
        orn = state[5]
        rot_matrix = p.getMatrixFromQuaternion(orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        T_EE = np.eye(4)
        T_EE[:3, :3] = rot_matrix
        T_EE[:3, 3] = pos

        self.T_EE = T_EE


    def EndSimulation(self):
