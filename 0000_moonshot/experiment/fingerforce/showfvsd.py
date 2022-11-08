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


if __name__ == "__main__":

    fvsd0 = pickle.load(open("cm/f_cm_0_30_0.pickle", "rb"))
    # fvsd1 = pickle.load(open("f_g_0_20_1.pickle", "rb"))
    # fvsd2 = pickle.load(open("f_g_0_20_2.pickle", "rb"))
    # fvsd3 = pickle.load(open("f_g_0_20_3.pickle", "rb"))
    # fvsd4 = pickle.load(open("f_g_0_20_4.pickle", "rb"))
    fig = plt.figure(1, figsize=(16, 9))
    x = [item[0] for item in fvsd0]

    y0 = [-item[1][2] for item in fvsd0]
    # z0 = [-item[2] for item in fvsd0]
    # y0_baseline =y0[0]
    # y0 = [item -y0_baseline for item in y0]

    # y1 = [-item[1][2] for item in fvsd1]
    # z1 = [-item[3] for item in fvsd1]
    # # y1_baseline = y1[0]
    # # y1 = [item - y1_baseline for item in y1]
    # #
    # y2 = [-item[1][2] for item in fvsd2]
    # z2 = [-item[3] for item in fvsd2]
    # # y2_baseline = y2[0]
    # # y2 = [item - y2_baseline for item in y2]
    #
    # y3 = [-item[1][2] for item in fvsd3]
    # z3 = [-item[3] for item in fvsd3]
    # # y3_baseline = y3[0]
    # # y3 = [item - y3_baseline for item in y3]
    #
    # y4 = [-item[1][2] for item in fvsd4]
    # z4 = [-item[3] for item in fvsd4]
    # # y4_baseline = y4[0]
    # # y4 = [item - y4_baseline for item in y4]
    # plt.subplot(211)
    plt.plot(x, y0)
    # plt.plot(x, y1)
    # plt.plot(x, y2)
    # plt.plot(x, y3)
    # plt.plot(x, y4)
    # plt.subplot(212)
    # plt.plot(x, z0)
    # plt.plot(x, z1)
    # plt.plot(x, z2)
    # plt.plot(x, z3)
    # plt.plot(x, z4)
    plt.show()