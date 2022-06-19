import numpy as np
from spatialmath import *
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import *
from roboticstoolbox.backends.swift import Swift
from roboticstoolbox.backends.VPython import VPython
import time

def zadanie_3():
    robot = rtb.models.DH.Panda()
    T = SE3(0.65, 0.2, 0.15)* SE3.Rx(np.pi)
    solution = robot.ikine_LM(T)
    T1 = SE3(0.75,0.2,0.15)* SE3.Rx(np.pi)
    solution1 = robot.ikine_LM(T1)
    r=0.1
    z=0.15
    t=np.linspace(0, 360, 60)
    via_pt = np.array([solution1.q])
    for i in t:
        x = (r * np.cos(i * np.pi / 180) + 0.65)
        y = (r * np.sin(i * np.pi / 180) + 0.2)
        T = SE3(x, y, z) * SE3.Rx(np.pi)
        solution_whole = robot.ikine_LMS(T)
        via_pt = np.append(via_pt, [solution_whole.q], axis=0)
    via_pt0 = (np.c_[robot.qz, solution.q, solution1.q]).T
    traj = mstraj(via_pt0, dt=0.05, tacc=0.20, qdmax=1.0)
    traj1 = mstraj(via_pt, dt=0.2, tacc=0.20, qdmax=4.0)
    traj2 = (np.c_[(traj.q).T, (traj1.q).T]).T
    robot.plot(traj2, backend='pyplot', limits=[-0.25, 1.25, -0.5, 0.5, 0, 1], movie='panda_pyplot.gif')
    rtb.qplot(traj2, block=True)

if __name__ == '__main__':
    zadanie_3()
