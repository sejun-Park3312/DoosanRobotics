from RobotSimulation import RobotSimulation
import time
import threading
import math as m
import numpy as np

lock = threading.Lock()
RS = RobotSimulation(lock)
RS_Thread = threading.Thread(target=RS.Simulation, daemon=True)
RS_Thread.start()


t = 0
while RS.Running:
    with lock:
        RS.q = np.array([1,1,1,1,1,1,1], dtype=float) * m.sin(t)
        RS.P_World2Capsule = np.array([0, 0, 0.5 * m.sin(t)])

    t = t + 0.1
    time.sleep(0.05)