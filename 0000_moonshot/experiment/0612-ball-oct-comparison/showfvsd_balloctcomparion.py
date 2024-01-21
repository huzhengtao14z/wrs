
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

if __name__ == "__main__":

    fvsd0_fg_0 = pickle.load(open("r_s1_cm_85_10_0.pickle", "rb"))
    fvsd1_fg_0 = pickle.load(open("r_s1_cm_85_10_1.pickle", "rb"))
    fvsd2_fg_0 = pickle.load(open("r_s1_cm_85_10_2.pickle", "rb"))

    fvsd0_fg_1 = pickle.load(open("r_s1_bcm_85_10_0.pickle", "rb"))
    fvsd1_fg_1 = pickle.load(open("r_s1_bcm_85_10_1.pickle", "rb"))
    fvsd2_fg_1 = pickle.load(open("r_s1_bcm_85_10_2.pickle", "rb"))

    fig = plt.figure(figsize=(16, 9))
    plt.subplots_adjust(left=0.25, right=0.95, top=0.95, bottom=0.21)
    ax = plt.gca()
    plt.xlim([0, 10])
    plt.ylim([0, 45])
    bwith = 2
    ax.spines["bottom"].set_linewidth(bwith)
    ax.spines['left'].set_linewidth(bwith)
    ax.spines['top'].set_linewidth(bwith)
    ax.spines['right'].set_linewidth(bwith)
    plt.tick_params(axis='both', direction='in', length=6, width=2, labelsize=30, pad=15)

    def case(fvsd0, fvsd1, fvsd2, color, rate =1.0):
        x = [item[0] for item in fvsd0]

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



        y_average = [(y0[i] + y1[i] + y2[i]) / 3 for i in range(11)]

        y_max = [max([y0[i], y1[i], y2[i]]) for i in range(11)]
        y_min = [min([y0[i], y1[i], y2[i]]) for i in range(11)]

        ax.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
        x_firsthalf = x[:11]
        ax.plot(x_firsthalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)

        y_average = [(y0[i] + y1[i] + y2[i]) / 3 for i in range(10, 21)]
        y_max = [max([y0[i], y1[i], y2[i]]) for i in range(10, 21)]
        y_min = [min([y0[i], y1[i], y2[i]]) for i in range(10, 21)]
        x_secondhalf = x[10:21]
        ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    case(fvsd0_fg_0, fvsd1_fg_0, fvsd2_fg_0, "red", rate = 7/7)
    case(fvsd0_fg_1, fvsd1_fg_1, fvsd2_fg_1, "deepskyblue", rate=7 / 7)
    # case(fvsd0_fg_bos, fvsd1_fg_bos, fvsd2_fg_bos, "gold", rate=8 / 7)
    # case(fvsd0_fg_cms, fvsd1_fg_cms, fvsd2_fg_cms, "navy", rate=8 / 7)
    # case(fvsd0_fg_rgs, fvsd1_fg_rgs, fvsd2_fg_rgs, "green", rate=8 / 7)
    # case(fvsd0_fg_65, fvsd1_fg_65, fvsd2_fg_65, fvsd3_fg_65, fvsd4_fg_65, "gold", rate = 6/7)
    # case(fvsd0_fg_85, fvsd1_fg_85, fvsd2_fg_85, fvsd3_fg_85, fvsd4_fg_85, "deepskyblue", rate = 6/7)




    ax.set_xlabel("Displacement/mm", fontsize=28)
    ax.set_ylabel("Force/N", fontsize=28)
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()