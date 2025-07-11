from RobotSimulation import RobotSimulation
import time
import threading
import math as m
import numpy as np

RS = RobotSimulation()
RS_Thread = threading.Thread(target=RS.Simulation, daemon=True)
RS_Thread.start()

lock = threading.Lock()
t = 0

while RS.Running:
    with lock:
        RS.q = np.array([1,1,1,1,1,1,1], dtype=float) * m.sin(t)
        RS.Get_Config()

    t = t + 0.1
    time.sleep(0.05)