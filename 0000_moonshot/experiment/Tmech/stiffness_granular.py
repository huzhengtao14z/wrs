
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

if __name__ == "__main__":

    fvsd0_fg_0 = pickle.load(open("../threepntbending/f_g_0_20_0.pickle", "rb"))
    fvsd1_fg_0 = pickle.load(open("../threepntbending/f_g_0_20_1.pickle", "rb"))
    fvsd2_fg_0 = pickle.load(open("../threepntbending/f_g_0_20_2.pickle", "rb"))
    fvsd3_fg_0 = pickle.load(open("../threepntbending/f_g_0_20_3.pickle", "rb"))
    fvsd4_fg_0 = pickle.load(open("../threepntbending/f_g_0_20_4.pickle", "rb"))

    fvsd0_fg_25 = pickle.load(open("../threepntbending/f_g_25_20_0.pickle", "rb"))
    fvsd1_fg_25 = pickle.load(open("../threepntbending/f_g_25_20_1.pickle", "rb"))
    fvsd2_fg_25 = pickle.load(open("../threepntbending/f_g_25_20_2.pickle", "rb"))
    fvsd3_fg_25 = pickle.load(open("../threepntbending/f_g_25_20_3.pickle", "rb"))
    fvsd4_fg_25 = pickle.load(open("../threepntbending/f_g_25_20_4.pickle", "rb"))

    fvsd0_fg_45 = pickle.load(open("../threepntbending/f_g_45_20_0.pickle", "rb"))
    fvsd1_fg_45 = pickle.load(open("../threepntbending/f_g_45_20_1.pickle", "rb"))
    fvsd2_fg_45 = pickle.load(open("../threepntbending/f_g_45_20_2.pickle", "rb"))
    fvsd3_fg_45 = pickle.load(open("../threepntbending/f_g_45_20_3.pickle", "rb"))
    fvsd4_fg_45 = pickle.load(open("../threepntbending/f_g_45_20_4.pickle", "rb"))

    fvsd0_fg_65 = pickle.load(open("../threepntbending/f_g_65_20_0.pickle", "rb"))
    fvsd1_fg_65 = pickle.load(open("../threepntbending/f_g_65_20_1.pickle", "rb"))
    fvsd2_fg_65 = pickle.load(open("../threepntbending/f_g_65_20_2.pickle", "rb"))
    fvsd3_fg_65 = pickle.load(open("../threepntbending/f_g_65_20_3.pickle", "rb"))
    fvsd4_fg_65 = pickle.load(open("../threepntbending/f_g_65_20_4.pickle", "rb"))

    fvsd0_fg_85 = pickle.load(open("../threepntbending/f_g_85_20_9.pickle", "rb"))
    fvsd1_fg_85 = pickle.load(open("../threepntbending/f_g_85_20_5.pickle", "rb"))
    fvsd2_fg_85 = pickle.load(open("../threepntbending/f_g_85_20_6.pickle", "rb"))
    fvsd3_fg_85 = pickle.load(open("../threepntbending/f_g_85_20_7.pickle", "rb"))
    fvsd4_fg_85 = pickle.load(open("../threepntbending/f_g_85_20_8.pickle", "rb"))

    fig = plt.figure(figsize=(16, 9))
    plt.subplots_adjust(left=0.25, right=0.95, top=0.95, bottom=0.21)
    ax = plt.gca()
    plt.xlim([0, 20])
    plt.ylim([0, 35])
    bwith = 2
    ax.spines["bottom"].set_linewidth(bwith)
    ax.spines['left'].set_linewidth(bwith)
    ax.spines['top'].set_linewidth(bwith)
    ax.spines['right'].set_linewidth(bwith)
    plt.tick_params(axis='both', direction='in', length=6, width=2, labelsize=30, pad=15)

    def case(fvsd0, fvsd1, fvsd2, fvsd3, fvsd4, color, rate =1.0):
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

        y3 = [-item[1][2]*rate for item in fvsd3]
        y3_baseline = y3[0]
        y3 = [item - y3_baseline for item in y3]

        y4 = [-item[1][2]*rate for item in fvsd4]
        y4_baseline = y4[0]
        y4 = [item - y4_baseline for item in y4]

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(21)]

        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(21)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(21)]

        ax.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
        x_firsthalf = x[:21]
        ax.plot(x_firsthalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(20, 41)]
        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(20, 41)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(20, 41)]
        x_secondhalf = x[20:41]
        ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    case(fvsd0_fg_0, fvsd1_fg_0, fvsd2_fg_0, fvsd3_fg_0, fvsd4_fg_0, "red", rate = 6/7)
    case(fvsd0_fg_25, fvsd1_fg_25, fvsd2_fg_25, fvsd3_fg_25, fvsd4_fg_25, "green", rate = 6/7)
    case(fvsd0_fg_45, fvsd1_fg_45, fvsd2_fg_45, fvsd3_fg_45, fvsd4_fg_45, "navy", rate = 6/7)
    case(fvsd0_fg_65, fvsd1_fg_65, fvsd2_fg_65, fvsd3_fg_65, fvsd4_fg_65, "gold", rate = 6/7)
    case(fvsd0_fg_85, fvsd1_fg_85, fvsd2_fg_85, fvsd3_fg_85, fvsd4_fg_85, "deepskyblue", rate = 6/7)




    ax.set_xlabel("Displacement/mm")
    ax.set_ylabel("Force/N")
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()