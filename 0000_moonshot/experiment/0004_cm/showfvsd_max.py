
import matplotlib.pyplot as plt
import pickle
import matplotlib.ticker as mtick
import numpy as np

if __name__ == "__main__":

    fvsd0_fg_0 = pickle.load(open("a90-77-10-3-0.pickle", "rb"))
    fvsd1_fg_0 = pickle.load(open("a90-77-10-3-1.pickle", "rb"))
    fvsd2_fg_0 = pickle.load(open("a90-77-10-3-2.pickle", "rb"))
    fvsd3_fg_0 = pickle.load(open("a90-77-10-3-3.pickle", "rb"))
    fvsd4_fg_0 = pickle.load(open("a90-77-10-3-4.pickle", "rb"))




    fig = plt.figure(1, figsize=(16, 9))
    ax = fig.add_subplot(111)
    plt.xlim([0, 10])

    def case(fvsd0, fvsd1, fvsd2, fvsd3, fvsd4, color, rate =1.0):
        x = [np.rad2deg(np.arctan((30)/200)-np.arctan((30-(3*item[0]))/200)) for item in fvsd0]

        torque_0 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd0]
        torque_1 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd1]
        torque_2 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd2]
        torque_3 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd3]
        torque_4 = [(-item[1][0] * 3 * (10 - item[0]) - 1*(item[1][2] * 200))*0.001 for item in fvsd4]

        y0 = torque_0
        y1 = torque_1
        y2 = torque_2
        y3 = torque_3
        y4 = torque_4

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(11)]

        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(11)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(11)]

        ax.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
        x_firsthalf = x[:11]
        ax.plot(x_firsthalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_firsthalf, y_min, y_max, color=color, alpha=0.2)

        y_average = [(y0[i] + y1[i] + y2[i] + y3[i] + y4[i]) / 5 for i in range(10, 21)]
        y_max = [max([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(10, 21)]
        y_min = [min([y0[i], y1[i], y2[i], y3[i], y4[i]]) for i in range(10, 21)]
        x_secondhalf = x[10:21]
        ax.plot(x_secondhalf, y_average, color=color, linewidth=3)
        plt.fill_between(x_secondhalf, y_min, y_max, color=color, alpha=0.2)

    case(fvsd0_fg_0, fvsd1_fg_0, fvsd2_fg_0, fvsd3_fg_0, fvsd4_fg_0, "red", rate = 6/7)
    # case(fvsd0_fg_25, fvsd1_fg_25, fvsd2_fg_25, fvsd3_fg_25, fvsd4_fg_25, "green", rate = 1)
    # case(fvsd0_fg_45, fvsd1_fg_45, fvsd2_fg_45, fvsd3_fg_45, fvsd4_fg_45, "navy", rate = 1)



    plt.xticks(fontsize = 20)
    plt.yticks(fontsize=20)
    ax.set_xlabel("Angle/degree", fontsize=20)
    ax.set_ylabel("Torque/Nm", fontsize=20)
    # ax.plot(x, y_max, linewidth=5)
    # ax.plot(x, y_min, linewidth=5)
    plt.show()