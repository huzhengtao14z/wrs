
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

if __name__ == "__main__":

    fvsd0_fg_0 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_0.pickle", "rb"))
    fvsd1_fg_0 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_0.pickle", "rb"))
    fvsd2_fg_0 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_2.pickle", "rb"))
    fvsd3_fg_0 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_3.pickle", "rb"))
    fvsd4_fg_0 = pickle.load(open("0117singlecore/p_s2_dcm_85_3_4.pickle", "rb"))

    fvsd0_fg_25 = pickle.load(open("0117singlecore/p_s2_cm_85_20_0.pickle", "rb"))
    fvsd1_fg_25 = pickle.load(open("0117singlecore/p_s2_cm_85_20_1.pickle", "rb"))
    fvsd2_fg_25 = pickle.load(open("0117singlecore/p_s2_cm_85_20_2.pickle", "rb"))
    fvsd3_fg_25 = pickle.load(open("0117singlecore/p_s2_cm_85_20_3.pickle", "rb"))
    fvsd4_fg_25 = pickle.load(open("0117singlecore/p_s2_cm_85_20_4.pickle", "rb"))
    # #
    fvsd0_fg_45 = pickle.load(open("0120thickcore/r_s2_cmthick_85_20_5.pickle", "rb"))
    fvsd1_fg_45 = pickle.load(open("0120thickcore/r_s2_cmthick_85_20_6.pickle", "rb"))
    fvsd2_fg_45 = pickle.load(open("0120thickcore/r_s2_cmthick_85_20_2.pickle", "rb"))
    fvsd3_fg_45 = pickle.load(open("0120thickcore/r_s2_cmthick_85_20_3.pickle", "rb"))
    fvsd4_fg_45 = pickle.load(open("0120thickcore/r_s2_cmthick_85_20_4.pickle", "rb"))
    #
    # fvsd0_fgb_45 = pickle.load(open("0731/f_gb_80_20_0.pickle", "rb"))
    # fvsd1_fgb_45 = pickle.load(open("0731/f_gb_80_20_1.pickle", "rb"))
    # fvsd2_fgb_45 = pickle.load(open("0731/f_gb_80_20_2.pickle", "rb"))
    # fvsd3_fgb_45 = pickle.load(open("0731/f_gb_80_20_3.pickle", "rb"))
    # fvsd4_fgb_45 = pickle.load(open("0731/f_gb_80_20_4.pickle", "rb"))
    #
    # fvsd0_fgs_45 = pickle.load(open("0731/f_gs_80_20_0.pickle", "rb"))
    # fvsd1_fgs_45 = pickle.load(open("0731/f_gs_80_20_1.pickle", "rb"))
    # fvsd2_fgs_45 = pickle.load(open("0731/f_gs_80_20_2.pickle", "rb"))
    # fvsd3_fgs_45 = pickle.load(open("0731/f_gs_80_20_3.pickle", "rb"))
    # fvsd4_fgs_45 = pickle.load(open("0731/f_gs_80_20_4.pickle", "rb"))



    fig = plt.figure(1, figsize=(16, 9))
    ax = fig.add_subplot(111)
    plt.xlim([0, 21])



    def case(fvsd0, fvsd1, fvsd2, fvsd3, fvsd4, color, rate =1.0):
        x = [item[0] for item in fvsd0]

        y0 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd0]

        y0_baseline = y0[0]
        y0 = [item - y0_baseline for item in y0]

        y1 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd1]
        y1_baseline = y1[0]
        y1 = [item - y1_baseline for item in y1]
        #
        y2 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd2]
        y2_baseline = y2[0]
        y2 = [item - y2_baseline for item in y2]

        y3 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd3]
        y3_baseline = y3[0]
        y3 = [item - y3_baseline for item in y3]

        y4 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd4]
        y4_baseline = y4[0]
        y4 = [item - y4_baseline for item in y4]

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(7)]

        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(7)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(7)]

        ax.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
        x_firsthalf = x[:7]
        ax.plot(x_firsthalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(6, 13)]
        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(6, 13)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(6, 13)]
        x_secondhalf = x[6:13]
        ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    def case_n(fvsd0, fvsd1, fvsd2, fvsd3, fvsd4, color, n = 20, rate =1.0):
        x = [item[0] for item in fvsd0]

        y0 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd0]
        y0_baseline = y0[0]
        y0 = [item - y0_baseline for item in y0]

        y1 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd1]
        y1_baseline = y1[0]
        y1 = [item - y1_baseline for item in y1]
        #
        y2 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd2]
        y2_baseline = y2[0]
        y2 = [item - y2_baseline for item in y2]

        y3 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd3]
        y3_baseline = y3[0]
        y3 = [item - y3_baseline for item in y3]

        y4 = [-item[1][2]*rate if -item[1][2] >= 0 else 0 for item in fvsd4]
        y4_baseline = y4[0]
        y4 = [item - y4_baseline for item in y4]

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range( n+1)]

        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range( n+1)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range( n+1)]

        ax.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
        x_firsthalf = x[:n+1]
        ax.plot(x_firsthalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(n, 2*n+1)]
        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(n, 2*n+1)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(n, 2*n+1)]
        x_secondhalf = x[n:2*n+1]
        ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    case(fvsd0_fg_0, fvsd1_fg_0, fvsd2_fg_0, fvsd3_fg_0, fvsd4_fg_0, "red", rate = 10/3)
    case_n(fvsd0_fg_25, fvsd1_fg_25, fvsd2_fg_25, fvsd3_fg_25, fvsd4_fg_25, "green", rate = 1)
    case_n(fvsd0_fg_45, fvsd1_fg_45, fvsd2_fg_45, fvsd3_fg_45, fvsd4_fg_45, "navy", n = 10, rate = 2*0.85)
    # case(fvsd0_fgb_45, fvsd1_fgb_45, fvsd2_fgb_45, fvsd3_fgb_45, fvsd4_fgb_45, "yellow", rate=1)
    # case(fvsd0_fgs_45, fvsd1_fgs_45, fvsd2_fgs_45, fvsd3_fgs_45, fvsd4_fgs_45, "brown", rate=1)


    ax.set_xlabel("Displacement/mm")
    ax.set_ylabel("Force/N")
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()