import numpy as np

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
import CheckJointsingle as checkangle


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

    output_fvsd_list = []
    # ftrot = np.eye(3)
    # def task():
    file_name = "a90-77-10-3-6.pickle"
    '''
    f:flat   b:bellow   
    cm:chain mail   g: granular
    first number: pressure
    second number: maximumdistance:
    third number: number of trails
    
    '''
    def downward(dis_step = 3, time_step = 0.5, dis_max = 10 ):
        dis = 0
        start_angle = checkangle.getVec()
        print(start_angle)
        while dis < dis_max:
            ft = np.asarray(rbtx.moniterft("lft", False))
            force = 0
            count = 10
            i = 0
            while i < count :
                force = force + ftrot.dot(ft[:3].T)
                i += 1
            angle_list = []
            for i in range(3):
                current_vec = checkangle.getVec()
                angle_list.append(rm.degree_betweenvector(start_angle, current_vec))
            angle = np.average(np.asarray(angle_list), axis = 0)
            print(angle)
            output_fvsd_list.append([dis,force/10, angle])
            eepos, eerot = rbt.getee(armname="lft")
            currentjnts = rbt.getarmjnts(armname="lft")
            eepos = eepos + np.array([0, 0, -1]) * dis_step
            newjnts = rbt.numikmsc(eepos, eerot, currentjnts, armname="lft")
            print(currentjnts)
            print(newjnts)
            rbt.movearmfk(newjnts, armname="lft")
            rbtx.movejntssgl(newjnts, armname="lft", wait=True)
            time.sleep(time_step)
            dis += 1
        while dis >0 :
            ft = np.asarray(rbtx.moniterft("lft", False))
            force = 0
            count = 10
            i = 0
            while i < count :
                force = force + ftrot.dot(ft[:3].T)
                i +=1
            angle_list = []
            for i in range(3):
                current_vec = checkangle.getVec()
                angle_list.append(rm.degree_betweenvector(start_angle, current_vec))
            angle = np.average(np.asarray(angle_list), axis=0)
            output_fvsd_list.append([dis,force/10, angle])
            eepos, eerot = rbt.getee(armname="lft")
            currentjnts = rbt.getarmjnts(armname="lft")
            eepos = eepos + np.array([0, 0, 1]) * dis_step
            newjnts = rbt.numikmsc(eepos, eerot, currentjnts, armname="lft")
            print(currentjnts)
            print(newjnts)
            rbt.movearmfk(newjnts, armname="lft")
            rbtx.movejntssgl(newjnts, armname="lft", wait=True)
            time.sleep(time_step)
            dis -= 1

        ft = np.asarray(rbtx.moniterft("lft", False))
        force = 0
        count = 10
        i = 0
        while i < count:
            force = force + ftrot.dot(ft[:3].T)
            i += 1
        angle_list = []
        for i in range(3):
            current_vec = checkangle.getVec()
            angle_list.append(rm.degree_betweenvector(start_angle, current_vec))
        angle = np.average(np.asarray(angle_list), axis=0)
        output_fvsd_list.append([dis, force / 10, angle])

        with open(file_name, "wb") as file:
            pickle.dump(output_fvsd_list, file)

    def impedance():
        rbtx.impedance(armname="lft")

    t2 = Thread(target=downward)
    t2.start()
    # t3 = Thread(target=impedance)
    # t3.start()
    rbtx.zerotcpforce("lft")
    startft = rbtx.moniterft("lft", False)
    startforce = startft[:3]
    starttorque = startft[3:]
    starttimespot = 0
    output_f_list = []
    output_t_list = []
    while True:
        timespot = time.time() - starttime
        ft = np.asarray(rbtx.moniterft("lft", False))
        force = ftrot.dot(ft[:3].T)
        torque = ftrot.dot(ft[3:].T)

        output_f_list.append([timespot, force])
        output_t_list.append([timespot, torque])

        plt.plot([starttimespot, timespot], [startforce[0], force[0]], color="red")
        plt.plot([starttimespot, timespot], [startforce[1], force[1]], color="green")
        plt.plot([starttimespot, timespot], [startforce[2], force[2]], color="blue")

        # timespot = time.time() - starttime
        # ft = np.asarray(rbtx.moniterft("lft", False))
        # force = ftrot.dot(ft[:3].T)
        #
        # plt.plot([starttimespot, timespot], [startforce[0], force[0]], color="red")
        # plt.plot([starttimespot, timespot], [startforce[1], force[1]], color="green")
        # plt.plot([starttimespot, timespot], [startforce[2], force[2]], color="blue")



        # plt.subplot(212)
        # plt.plot([starttimespot, timespot], [starttorque[0], torque[0]], color="red")
        # plt.plot([starttimespot, timespot], [starttorque[1], torque[1]], color="green")
        # plt.plot([starttimespot, timespot], [starttorque[2], torque[2]], color="blue")
        # starttorque = torque
        startforce = force
        starttimespot = timespot

        plt.pause(0.001)
        # time.sleep(0.1)
        # plt.plot(ft)

