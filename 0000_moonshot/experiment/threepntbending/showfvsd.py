
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

if __name__ == "__main__":

    fvsd0_fg = pickle.load(open("f_g_0_20_0.pickle", "rb"))
    fvsd1_fg = pickle.load(open("f_g_0_20_1.pickle", "rb"))
    fvsd2_fg = pickle.load(open("f_g_0_20_2.pickle", "rb"))
    fvsd3_fg = pickle.load(open("f_g_0_20_3.pickle", "rb"))
    fvsd4_fg = pickle.load(open("f_g_0_20_4.pickle", "rb"))

    fvsd0_fcm = pickle.load(open("f_cm_0_20_0.pickle", "rb"))
    fvsd1_fcm = pickle.load(open("f_cm_0_20_1.pickle", "rb"))
    fvsd2_fcm = pickle.load(open("f_cm_0_20_2.pickle", "rb"))
    fvsd3_fcm = pickle.load(open("f_cm_0_20_3.pickle", "rb"))
    fvsd4_fcm = pickle.load(open("f_cm_0_20_4.pickle", "rb"))

    fvsd0_bcm = pickle.load(open("b_cm_0_20_0.pickle", "rb"))
    fvsd1_bcm = pickle.load(open("b_cm_0_20_1.pickle", "rb"))
    fvsd2_bcm = pickle.load(open("b_cm_0_20_2.pickle", "rb"))
    fvsd3_bcm = pickle.load(open("b_cm_0_20_3.pickle", "rb"))
    fvsd4_bcm = pickle.load(open("b_cm_0_20_4.pickle", "rb"))





    fig = plt.figure(1, figsize=(16, 9))
    ax = fig.add_subplot(111)
    plt.xlim([0, 21])



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
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.1)

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(20, 41)]
        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(20, 41)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(20, 41)]
        x_secondhalf = x[20:41]
        ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.1)

    case(fvsd0_fg, fvsd1_fg, fvsd2_fg, fvsd3_fg, fvsd4_fg, "red", rate = 6/7)
    case(fvsd0_fcm, fvsd1_fcm, fvsd2_fcm, fvsd3_fcm, fvsd4_fcm, "green")
    case(fvsd0_bcm, fvsd1_bcm, fvsd2_bcm, fvsd3_bcm, fvsd4_bcm, "navy")





    ax.set_xlabel("Displacement/mm")
    ax.set_ylabel("Force/N")
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()