
import pickle
import matplotlib.pyplot as plt



if __name__ == "__main__":

    fvsd0 = pickle.load(open("b_cm_85_20_4.pickle", "rb"))
    fvsd1 = pickle.load(open("b_cm_85_20_5.pickle", "rb"))
    fvsd2 = pickle.load(open("b_cm_85_20_6.pickle", "rb"))
    fvsd3 = pickle.load(open("b_cm_85_20_7.pickle", "rb"))
    fvsd4 = pickle.load(open("b_cm_85_20_8.pickle", "rb"))

    fig = plt.figure(1, figsize=(16, 9))
    x = [item[0] for item in fvsd0]

    y0 = [-item[1][2] for item in fvsd0]
    y0_baseline =y0[0]
    y0 = [item -y0_baseline for item in y0]

    y1 = [-item[1][2] for item in fvsd1]
    y1_baseline = y1[0]
    y1 = [item - y1_baseline for item in y1]
    #
    y2 = [-item[1][2] for item in fvsd2]
    y2_baseline = y2[0]
    y2 = [item - y2_baseline for item in y2]

    y3 = [-item[1][2] for item in fvsd3]
    y3_baseline = y3[0]
    y3 = [item - y3_baseline for item in y3]

    y4 = [-item[1][2] for item in fvsd4]
    y4_baseline = y4[0]
    y4 = [item - y4_baseline for item in y4]

    plt.plot(x, y0)
    plt.plot(x, y1)
    plt.plot(x, y2)
    plt.plot(x, y3)
    plt.plot(x, y4)

    plt.show()