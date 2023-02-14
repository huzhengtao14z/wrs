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

    # fvsd0 = pickle.load(open("0731/f_gs_0_20_0.pickle", "rb"))
    # fvsd1 = pickle.load(open("0731/f_gs_0_20_1.pickle", "rb"))
    # fvsd2 = pickle.load(open("0731/f_gs_0_20_2.pickle", "rb"))
    # fvsd3 = pickle.load(open("0731/f_gs_0_20_3.pickle", "rb"))
    # fvsd4 = pickle.load(open("0731/f_gs_0_20_4.pickle", "rb"))

    # fvsd0 = pickle.load(open("0731/f_gb_0_20_0.pickle", "rb"))
    # fvsd1 = pickle.load(open("0731/f_gb_0_20_1.pickle", "rb"))
    # fvsd2 = pickle.load(open("0731/f_gb_0_20_2.pickle", "rb"))
    # fvsd3 = pickle.load(open("0731/f_gb_0_20_3.pickle", "rb"))
    # fvsd4 = pickle.load(open("0731/f_gb_0_20_4.pickle", "rb"))

    # fvsd0 = pickle.load(open("0818/b_cm_88_20_0.pickle", "rb"))
    # fvsd1 = pickle.load(open("0818/b_cm_88_20_1.pickle", "rb"))
    # fvsd2 = pickle.load(open("0818/b_cm_88_20_2.pickle", "rb"))
    # fvsd3 = pickle.load(open("0818/b_cm_88_20_3.pickle", "rb"))
    # fvsd4 = pickle.load(open("0818/b_cm_88_20_4.pickle", "rb"))

    # fvsd0 = pickle.load(open("0818/b_gs_80_20_3.pickle", "rb"))
    # fvsd1 = pickle.load(open("0818/b_gs_80_20_4.pickle", "rb"))
    # fvsd2 = pickle.load(open("0818/b_gs_80_20_5.pickle", "rb"))
    # fvsd3 = pickle.load(open("0818/b_gs_80_20_6.pickle", "rb"))
    # fvsd4 = pickle.load(open("0818/b_gs_80_20_7.pickle", "rb"))
    #
    # fvsd0 = pickle.load(open("0818/b_gs_80_20_3.pickle", "rb"))
    # fvsd1 = pickle.load(open("0818/b_gs_80_20_4.pickle", "rb"))
    # fvsd2 = pickle.load(open("0818/b_gs_80_20_5.pickle", "rb"))
    # fvsd3 = pickle.load(open("0818/b_gs_80_20_6.pickle", "rb"))
    # fvsd4 = pickle.load(open("0818/b_gs_80_20_7.pickle", "rb"))

    # fvsd0 = pickle.load(open("0731/f_cm_80_20_0.pickle", "rb"))
    # fvsd1 = pickle.load(open("0731/f_cm_80_20_1.pickle", "rb"))
    # fvsd2 = pickle.load(open("0731/f_cm_80_20_2.pickle", "rb"))
    # fvsd3 = pickle.load(open("0731/f_cm_80_20_3.pickle", "rb"))
    # fvsd4 = pickle.load(open("0731/f_cm_80_20_4.pickle", "rb"))

    # fvsd0 = pickle.load(open("0731/f_gb_80_20_0.pickle", "rb"))
    # fvsd1 = pickle.load(open("0731/f_gb_80_20_1.pickle", "rb"))
    # fvsd2 = pickle.load(open("0731/f_gb_80_20_2.pickle", "rb"))
    # fvsd3 = pickle.load(open("0731/f_gb_80_20_3.pickle", "rb"))
    # fvsd4 = pickle.load(open("0731/f_gb_80_20_4.pickle", "rb"))

    fvsd0 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_0.pickle", "rb"))
    # fvsd1 = pickle.load(open("0117singlecore/p_s2_cm_0_20_1.pickle", "rb"))
    # fvsd2 = pickle.load(open("0117singlecore/p_s2_cm_0_20_2.pickle", "rb"))
    # fvsd3 = pickle.load(open("0117singlecore/p_s2_cm_0_20_3.pickle", "rb"))
    # fvsd4 = pickle.load(open("0117singlecore/p_s2_cm_0_20_4.pickle", "rb"))

    fvsd1 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_1.pickle", "rb"))
    fvsd2 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_2.pickle", "rb"))
    fvsd3 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_3.pickle", "rb"))
    fvsd4 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_4.pickle", "rb"))

    fig = plt.figure(1, figsize=(16, 9))
    x = [item[0] for item in fvsd0]

    y0 = [-item[1][2]*(10/3) for item in fvsd0]
    y0_baseline =y0[0]
    y0 = [item -y0_baseline for item in y0]

    print(y0)
    y1 = [-item[1][2]*(10/3) for item in fvsd1]
    y1_baseline = y1[0]
    y1 = [item - y1_baseline for item in y1]
    #
    y2 = [-item[1][2]*(10/3) for item in fvsd2]
    y2_baseline = y2[0]
    y2 = [item - y2_baseline for item in y2]

    y3 = [-item[1][2]*(10/3) for item in fvsd3]
    y3_baseline = y3[0]
    y3 = [item - y3_baseline for item in y3]

    y4 = [-item[1][2]*(10/3) for item in fvsd4]
    y4_baseline = y4[0]
    y4 = [item - y4_baseline for item in y4]

    plt.plot(x,y0, color = 'r')
    plt.plot(x, y1)
    plt.plot(x, y2)
    plt.plot(x, y3)
    plt.plot(x, y4)

    plt.show()