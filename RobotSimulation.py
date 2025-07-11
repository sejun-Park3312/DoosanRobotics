import pybullet as p
import pybullet_data
import time
import math as m
import numpy as np
import threading

class RobotSimulaition:
    def __init__(self):

        self.Running = True
        self.physicsClient = p.connect(p.GUI)
        self.robotId = p.loadURDF(r"C:\Users\qkrtp.DESKTOP-60UHPRL\PycharmProjects\DoosanRobotics\robot.urdf",useFixedBase=True)
        self.HomePosition = np.array([0, 0, m.pi/2, 0, m.pi/2, 0])
        self.T_EE = None
        self.q = self.HomePosition
        self.Get_Config()
        self.Get_T_EE()
        self.lock = threading.Lock()


    def Get_Config(self):
        for j in range(6):
            p.resetJointState(self.robotId, j, self.q[j])


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


    def StartSimulation(self):
        while self.Running:

            with self.lock:
                self.Get_Config()
            p.stepSimulation()
            time.sleep(1. / 240.)

        self.EndSimulation()

    def EndSimulation(self):
        p.disconnect()