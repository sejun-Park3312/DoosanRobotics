import pybullet as p
import pybullet_data
import time
import math

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(r"C:\Users\qkrtp.DESKTOP-60UHPRL\PycharmProjects\DoosanRobotics\robot.urdf", useFixedBase=True)

num_joints = p.getNumJoints(robotId)
print(f"Number of joints: {num_joints}")

p.setGravity(0, 0, 0)

# 관절 각도 시퀀스 예시
t = 0
while True:
    # 예: joint 0 ~ 5까지 사인파로 움직임
    for j in range(num_joints):
        angle = 0.5 * math.sin(t)
        p.resetJointState(robotId, j, angle)
    t += 0.01
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
