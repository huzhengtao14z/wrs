import motionplanner.motion_planner as m_planner
import motionplanner.rbtx_motion_planner as m_planner_x
# import motionplanner.rbtx_motion_planner as m_planner_x
import hufunctions
import utils.phoxi as phoxi
import utils.phoxi_locator as pl
from utils.run_script_utils import *
import localenv.envloader as el
import math
import pickle
import re
import time
import os
import config
from socket import socket, AF_INET, SOCK_STREAM

import matplotlib.pyplot as plt
import utiltools.robotmath as rm
from direct.stdpy import threading
from threading import Thread

if __name__ == '__main__':
    '''
    set up env and param
    '''
    base, env = el.loadEnv_wrs()
    rbt, rbtmg, rbtball = el.loadUr3e()
    # rbt.opengripper(armname="rgt")
    # rbt.opengripper(armname="lft")
    rbtx = el.loadUr3ex(rbt)
    '''
    init class
    '''
    # mp_lft = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="lft")
    # mp_lftx = m_planner_x.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="lft")
    # mp_rgt = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="rgt")
    # mp_rgtx = m_planner_x.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="rgt")
    # mp_lftx.goto_init_x()
    # mp_rgtx.goto_init_x()

    # ftrot = np.linalg.inv(np.dot(rm.rodrigues((0, 1, 0), -90), rm.rodrigues((1, 0, 0), 90)))
    ftrot = np.dot(rm.rodrigues((0, 1, 0), 90), rm.rodrigues((0, 0, 1), -90))
    rbtx.moniterft("lft", continuous=False)
    ini_jnts = rbtx.getjnts("lft")
    rbt.movearmfk(ini_jnts, armname="lft")
    ftrot = np.linalg.inv(rbt.gettcp("lft")[1]).dot(ftrot)

    fig = plt.figure(1, figsize=(16, 9))
    plt.ion()
    plt.show()

    # plt.yticks((-3,3))
    starttime = time.time()
    k = 0


    # ftrot = np.eye(3)
    # def task():
    #




    def impedance():

        rbtx.impedance(armname="lft")

    t2 = Thread(target=impedance)

    # start the threads
    # t1.start()
    t2.start()

    rbtx.zerotcpforce("lft")
    startft = rbtx.moniterft("lft", False)
    startforce = startft[:3]
    starttorque = startft[3:]
    starttimespot = 0
    while True:
        plt.subplot(211)
        timespot = time.time() - starttime
        ft = np.asarray(rbtx.moniterft("lft", False))
        force = ftrot.dot(ft[:3].T)
        torque = ftrot.dot(ft[3:].T)
        print(ft)
        plt.plot([starttimespot, timespot], [startforce[0], force[0]], color="red")
        plt.plot([starttimespot, timespot], [startforce[1], force[1]], color="green")
        plt.plot([starttimespot, timespot], [startforce[2], force[2]], color="blue")
        plt.subplot(212)
        plt.plot([starttimespot, timespot], [starttorque[0], torque[0]], color="red")
        plt.plot([starttimespot, timespot], [starttorque[1], torque[1]], color="green")
        plt.plot([starttimespot, timespot], [starttorque[2], torque[2]], color="blue")
        starttorque = torque
        startforce = force
        starttimespot = timespot
        # ini_jnts[5]+=1
        # print(goal_jnts)
        # rbtx.movejntssgl(ini_jnts, armname="lft")
        # ini_jnts = goal_jnts
        plt.pause(0.001)
        # time.sleep(0.1)
        plt.plot(ft)
    # thread1 = myThread(1, "Thread-1", 1)
    # thread2 = myThread(2, "Thread-2", 2)
    #
    #
    # thread1.start()
    # thread2.start()

    # base.run()
