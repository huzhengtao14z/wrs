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
    file_name = "iks.pickle"
    ini_jnts = rbtx.getjnts("lft")
    rbt.movearmfk(ini_jnts, armname="lft")
    eepos, eerot = rbt.getee(armname="lft")
    currentjnts = rbt.getarmjnts(armname="lft")

    ik_list = []
    def downward(dis_step=1, time_step=0.5, dis_max=20):
        dis = 0
        # while True:
        #
        #     jnts = rbtx.getjnts("lft")
        #     rbt.movearmfk(jnts, "lft")
        #     pos, rot = rbt.gettcp("lft")
        #     pos = pos + np.array([0,0,1])
        #     next_jnts = rbt.numik(pos, rot,  seedjntagls = jnts, armname="lft")
        #     rbt.movearmfk(next_jnts,"lft")
        #     rbtx.movejntsall(next_jnts,"lft")
        #     time.sleep(0.1)
        while dis < dis_max:
            eepos, eerot = rbt.getee(armname="lft")
            currentjnts = rbt.getarmjnts(armname="lft")
            eepos = eepos + np.array([0, 0, -1]) * dis_step
            newjnts = rbt.numikmsc(eepos, eerot, currentjnts, armname="lft")
            print(currentjnts)
            print(newjnts)
            rbt.movearmfk(newjnts, armname="lft")
            rbtx.movejntssgl(newjnts, armname="lft", wait=True)
            time.sleep(time_step)
            ik_list.append(newjnts)
            dis += 1

        while dis > 0:

            eepos, eerot = rbt.getee(armname="lft")
            currentjnts = rbt.getarmjnts(armname="lft")
            eepos = eepos + np.array([0, 0, 1]) * dis_step
            newjnts = rbt.numikmsc(eepos, eerot, currentjnts, armname="lft")
            print(currentjnts)
            print(newjnts)
            rbt.movearmfk(newjnts, armname="lft")
            rbtx.movejntssgl(newjnts, armname="lft", wait=True)
            time.sleep(time_step)
            ik_list.append(newjnts)
            dis -= 1

        with open(file_name, "wb") as file:
            pickle.dump(ik_list, file)

    downward()