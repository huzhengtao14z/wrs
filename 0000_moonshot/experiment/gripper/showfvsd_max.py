
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

if __name__ == "__main__":

    af = pickle.load(open("10-20n-1.pickle", "rb"))
    # fvsd1_fg_0 = pickle.load(open("a90-77-10-3-1.pickle", "rb"))
    # fvsd2_fg_0 = pickle.load(open("a90-77-10-3-2.pickle", "rb"))
    # fvsd3_fg_0 = pickle.load(open("a90-77-10-3-3.pickle", "rb"))
    # fvsd4_fg_0 = pickle.load(open("a90-77-10-3-4.pickle", "rb"))




    fig = plt.figure(1, figsize=(9, 9))
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)


    def case(af, color, rate =1.0):
        # x = [np.rad2deg(np.arctan((30)/200)-np.arctan((30-(3*item[0]))/200)) for item in fvsd0]
        x = np.linspace(0,(len(af[0])-1)/30,len(af[0]))
        # torque_0 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd0]
        # torque_1 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd1]
        # torque_2 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd2]
        # torque_3 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd3]
        # torque_4 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd4]

        y0 = [-item[1][2] for item in af[2]]
        y1 = [item[1][1] for item in af[0]]
        # y1 = af[1]
        # y2 = af[2]
        # y3 = af[3]

        # y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(11)]
        #
        # y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(11)]
        # y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(11)]

        # ax.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
        # x_firsthalf = x[:11]
        # ax1.plot(x,y0, color="g", linewidth=3)
        ax2.plot(x,y1, color=color, linewidth=3)

        ax1.plot(x, y0, color="g", linewidth=3)
        # ax.plot(x,y2, color=color, linewidth=3)
        # ax.plot(x,y3, color=color, linewidth=3)
        # plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)
        #
        # y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(10, 21)]
        # y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(10, 21)]
        # y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(10, 21)]
        # x_secondhalf = x[10:21]
        # ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        # plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    case(af, "red", rate = 6/7)
    # case(fvsd0_fg_25, fvsd1_fg_25, fvsd2_fg_25, fvsd3_fg_25, fvsd4_fg_25, "green", rate = 1)
    # case(fvsd0_fg_45, fvsd1_fg_45, fvsd2_fg_45, fvsd3_fg_45, fvsd4_fg_45, "navy", rate = 1)



    ax1.tick_params(labelsize=18)
    ax2.tick_params(labelsize=18)
    # plt.yticks(fontsize=20)
    # plt.xticks(fontsize=20)
    # plt.yticks(fontsize=20)
    ax1.set_xlabel("Time", fontsize=20)
    ax1.set_ylabel("Force/N", fontsize=20)
    ax2.set_xlabel("Time", fontsize=20)
    ax2.set_ylabel("Force/N", fontsize=20)
    ax1.set_xlim([0, 46/30])
    ax2.set_xlim([0, 46/30])
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()