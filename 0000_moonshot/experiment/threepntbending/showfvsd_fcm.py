
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

if __name__ == "__main__":

    fvsd0_fg_0 = pickle.load(open("f_cm_0_20_0.pickle", "rb"))
    fvsd1_fg_0 = pickle.load(open("f_cm_0_20_1.pickle", "rb"))
    fvsd2_fg_0 = pickle.load(open("f_cm_0_20_2.pickle", "rb"))
    fvsd3_fg_0 = pickle.load(open("f_cm_0_20_3.pickle", "rb"))
    fvsd4_fg_0 = pickle.load(open("f_cm_0_20_4.pickle", "rb"))

    fvsd0_fg_25 = pickle.load(open("f_cm_24_20_0.pickle", "rb"))
    fvsd1_fg_25 = pickle.load(open("f_cm_24_20_1.pickle", "rb"))
    fvsd2_fg_25 = pickle.load(open("f_cm_24_20_2.pickle", "rb"))
    fvsd3_fg_25 = pickle.load(open("f_cm_24_20_3.pickle", "rb"))
    fvsd4_fg_25 = pickle.load(open("f_cm_24_20_4.pickle", "rb"))

    fvsd0_fg_45 = pickle.load(open("f_cm_45_20_0.pickle", "rb"))
    fvsd1_fg_45 = pickle.load(open("f_cm_45_20_1.pickle", "rb"))
    fvsd2_fg_45 = pickle.load(open("f_cm_45_20_2.pickle", "rb"))
    fvsd3_fg_45 = pickle.load(open("f_cm_45_20_3.pickle", "rb"))
    fvsd4_fg_45 = pickle.load(open("f_cm_45_20_4.pickle", "rb"))

    fvsd0_fg_65 = pickle.load(open("f_cm_64_20_0.pickle", "rb"))
    fvsd1_fg_65 = pickle.load(open("f_cm_64_20_1.pickle", "rb"))
    fvsd2_fg_65 = pickle.load(open("f_cm_64_20_2.pickle", "rb"))
    fvsd3_fg_65 = pickle.load(open("f_cm_64_20_3.pickle", "rb"))
    fvsd4_fg_65 = pickle.load(open("f_cm_64_20_4.pickle", "rb"))

    fvsd0_fg_85 = pickle.load(open("f_cm_85_20_0.pickle", "rb"))
    fvsd1_fg_85 = pickle.load(open("f_cm_85_20_1.pickle", "rb"))
    fvsd2_fg_85 = pickle.load(open("f_cm_85_20_2.pickle", "rb"))
    fvsd3_fg_85 = pickle.load(open("f_cm_85_20_3.pickle", "rb"))
    fvsd4_fg_85 = pickle.load(open("f_cm_85_20_4.pickle", "rb"))



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
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(20, 41)]
        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(20, 41)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(20, 41)]
        x_secondhalf = x[20:41]
        ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    case(fvsd0_fg_0, fvsd1_fg_0, fvsd2_fg_0, fvsd3_fg_0, fvsd4_fg_0, "red", rate = 6/7)
    case(fvsd0_fg_25, fvsd1_fg_25, fvsd2_fg_25, fvsd3_fg_25, fvsd4_fg_25, "green")
    case(fvsd0_fg_45, fvsd1_fg_45, fvsd2_fg_45, fvsd3_fg_45, fvsd4_fg_45, "navy")
    case(fvsd0_fg_65, fvsd1_fg_65, fvsd2_fg_65, fvsd3_fg_65, fvsd4_fg_65, "gold")
    case(fvsd0_fg_85, fvsd1_fg_85, fvsd2_fg_85, fvsd3_fg_85, fvsd4_fg_85, "deepskyblue")




    ax.set_xlabel("Displacement/mm")
    ax.set_ylabel("Force/N")
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()