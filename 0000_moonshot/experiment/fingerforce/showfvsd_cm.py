import matplotlib
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

# matplotlib.RcParams['font.family'] = 'Times New Roman'
# matplotlib.RcParams['font.size'] = 35
# matplotlib.RcParams['legend.markerscale'] = 0.5

if __name__ == "__main__":

    # fvsd0_fg_0 = pickle.load(open("cm/f_cm_90_30_0.pickle", "rb"))
    # fvsd1_fg_0 = pickle.load(open("cm/f_cm_90_30_1.pickle", "rb"))
    # fvsd2_fg_0 = pickle.load(open("cm/f_cm_90_30_2.pickle", "rb"))
    # fvsd3_fg_0 = pickle.load(open("cm/f_cm_90_30_3.pickle", "rb"))
    # fvsd4_fg_0 = pickle.load(open("cm/f_cm_90_30_4.pickle", "rb"))
    #
    # fvsd0_fg_25 = pickle.load(open("cm/f_cm_60_30_0.pickle", "rb"))
    # fvsd1_fg_25 = pickle.load(open("cm/f_cm_60_30_1.pickle", "rb"))
    # fvsd2_fg_25 = pickle.load(open("cm/f_cm_60_30_2.pickle", "rb"))
    # fvsd3_fg_25 = pickle.load(open("cm/f_cm_60_30_3.pickle", "rb"))
    # fvsd4_fg_25 = pickle.load(open("cm/f_cm_60_30_4.pickle", "rb"))
    #
    # fvsd0_fg_45 = pickle.load(open("cm/f_cm_30_30_0.pickle", "rb"))
    # fvsd1_fg_45 = pickle.load(open("cm/f_cm_30_30_1.pickle", "rb"))
    # fvsd2_fg_45 = pickle.load(open("cm/f_cm_30_30_2.pickle", "rb"))
    # fvsd3_fg_45 = pickle.load(open("cm/f_cm_30_30_3.pickle", "rb"))
    # fvsd4_fg_45 = pickle.load(open("cm/f_cm_30_30_4.pickle", "rb"))
    #
    # fvsd0_fgb_45 = pickle.load(open("cm/f_cm_0_30_0.pickle", "rb"))
    # fvsd1_fgb_45 = pickle.load(open("cm/f_cm_0_30_1.pickle", "rb"))
    # fvsd2_fgb_45 = pickle.load(open("cm/f_cm_0_30_2.pickle", "rb"))
    # fvsd3_fgb_45 = pickle.load(open("cm/f_cm_0_30_3.pickle", "rb"))
    # fvsd4_fgb_45 = pickle.load(open("cm/f_cm_0_30_4.pickle", "rb"))

    fvsd0_fg_0 = pickle.load(open("gs/f_gs_90_30_0.pickle", "rb"))
    fvsd1_fg_0 = pickle.load(open("gs/f_gs_90_30_1.pickle", "rb"))
    fvsd2_fg_0 = pickle.load(open("gs/f_gs_90_30_2.pickle", "rb"))
    fvsd3_fg_0 = pickle.load(open("gs/f_gs_90_30_3.pickle", "rb"))
    fvsd4_fg_0 = pickle.load(open("gs/f_gs_90_30_4.pickle", "rb"))

    fvsd0_fg_25 = pickle.load(open("gs/f_gs_60_30_0.pickle", "rb"))
    fvsd1_fg_25 = pickle.load(open("gs/f_gs_60_30_1.pickle", "rb"))
    fvsd2_fg_25 = pickle.load(open("gs/f_gs_60_30_2.pickle", "rb"))
    fvsd3_fg_25 = pickle.load(open("gs/f_gs_60_30_3.pickle", "rb"))
    fvsd4_fg_25 = pickle.load(open("gs/f_gs_60_30_4.pickle", "rb"))

    fvsd0_fg_45 = pickle.load(open("gs/f_gs_30_30_0.pickle", "rb"))
    fvsd1_fg_45 = pickle.load(open("gs/f_gs_30_30_1.pickle", "rb"))
    fvsd2_fg_45 = pickle.load(open("gs/f_gs_30_30_2.pickle", "rb"))
    fvsd3_fg_45 = pickle.load(open("gs/f_gs_30_30_3.pickle", "rb"))
    fvsd4_fg_45 = pickle.load(open("gs/f_gs_30_30_4.pickle", "rb"))

    fvsd0_fgb_45 = pickle.load(open("gs/f_gs_0_30_0.pickle", "rb"))
    fvsd1_fgb_45 = pickle.load(open("gs/f_gs_0_30_1.pickle", "rb"))
    fvsd2_fgb_45 = pickle.load(open("gs/f_gs_0_30_2.pickle", "rb"))
    fvsd3_fgb_45 = pickle.load(open("gs/f_gs_0_30_3.pickle", "rb"))
    fvsd4_fgb_45 = pickle.load(open("gs/f_gs_0_30_4.pickle", "rb"))

    # fvsd0_fgs_45 = pickle.load(open("0731/f_gs_80_20_0.pickle", "rb"))
    # fvsd1_fgs_45 = pickle.load(open("0731/f_gs_80_20_1.pickle", "rb"))
    # fvsd2_fgs_45 = pickle.load(open("0731/f_gs_80_20_2.pickle", "rb"))
    # fvsd3_fgs_45 = pickle.load(open("0731/f_gs_80_20_3.pickle", "rb"))
    # fvsd4_fgs_45 = pickle.load(open("0731/f_gs_80_20_4.pickle", "rb"))



    fig = plt.figure(figsize=(9, 9))
    plt.subplots_adjust(left=0.25, right=0.95, top = 0.95, bottom=0.21)
    ax = plt.gca()
    plt.xlim([0, 30])
    plt.ylim([0, 4.5])
    bwith = 2
    ax.spines["bottom"].set_linewidth(bwith)
    ax.spines['left'].set_linewidth(bwith)
    ax.spines['top'].set_linewidth(bwith)
    ax.spines['right'].set_linewidth(bwith)

    plt.tick_params(axis = 'both', direction = 'in',length = 6 , width = 2,  labelsize = 30, pad = 15)

    def case(fvsd0, fvsd1, fvsd2, fvsd3, color, rate =1.0):
        x = [item[0]*3 for item in fvsd0]

        y0 = [-item[1][2]*rate for item in fvsd0]
        y0_baseline = y0[0]
        y0 = [item - y0_baseline for item in y0]

        y1 = [-item[1][2]*rate for item in fvsd1]
        y1_baseline = y1[0]
        y1 = [item - y1_baseline for item in y1]
        #
        y2 = [-item[1][2]*rate for item in fvsd2]
        y2_baseline = y2[0]
        y2 = [item - y2_baseline for item in y2]

        y3 = [-item[1][2]*rate for item in fvsd3]
        y3_baseline = y3[0]
        y3 = [item - y3_baseline for item in y3]

        # y4 = [-item[1][2]*rate for item in fvsd4]
        # y4_baseline = y4[0]
        # y4 = [item - y4_baseline for item in y4]

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i]) / 4 for i in range(11)]

        y_max = [max([y0[i], y1[i], y2[i], y3[i]]) for i in range(11)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i]]) for i in range(11)]

        ax.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
        x_firsthalf = x[:11]
        ax.plot(x_firsthalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)

        # y_average = [(y0[i] + y1[i] + y2[i] + y3[i] ) / 4 for i in range(10, 21)]
        # y_max = [max([y0[i], y1[i], y2[i], y3[i]]) for i in range(10, 21)]
        # y_min = [min([y0[i], y1[i], y2[i], y3[i]]) for i in range(10, 21)]
        # x_secondhalf = x[10:21]
        # ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        # plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    # case(fvsd0_fg_0, fvsd1_fg_0, fvsd2_fg_0, fvsd3_fg_0, "skyblue", rate = 1)
    # case(fvsd0_fg_25, fvsd1_fg_25, fvsd2_fg_25, fvsd3_fg_25, "green", rate = 1)
    # case(fvsd0_fg_45, fvsd1_fg_45, fvsd2_fg_45, fvsd3_fg_45, "navy", rate = 1)
    case(fvsd0_fgb_45, fvsd1_fgb_45, fvsd2_fgb_45, fvsd3_fgb_45, "gold", rate=1)
    #90
    # x_exp = [0, 3.821, 7.64,11.46,15.38,19.11,22.93,25.98, 30]
    # y_exp = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.4, 4]
    # #60
    # x_exp = [0,4.989, 9.977, 14.97, 19.95, 24.94, 29.93]
    # y_exp = [0,0.5, 1, 1.5, 2, 2.5, 3]
    # #30
    # x_exp = [0,7.5, 15, 22.5, 30]
    # y_exp = [0,0.5, 1, 1.5, 2]
    # #0
    # x_exp = [0,8.73,16.27,24.4,32.53,40.66]
    # y_exp = [0,0.2,0.4,0.6,0.8,1]

    # 90
    # x_exp = [0, 27.4,30.82]
    # y_exp = [0, 4,4.5]
    # #60
    # x_exp = [0,32.99]
    # y_exp = [0,3.5]
    # #30
    # x_exp = [0,37.7]
    # y_exp = [0,2.5]
    # #0
    x_exp = [0,38.03]
    y_exp = [0,1]


    ax.plot(x_exp, y_exp, color='red', linewidth=3)


    # case(fvsd0_fgs_45, fvsd1_fgs_45, fvsd2_fgs_45, fvsd3_fgs_45, fvsd4_fgs_45, "brown", rate=1)



    ax.set_xlabel("Displacement/mm")
    ax.set_ylabel("Force/N")
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()