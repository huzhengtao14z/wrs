# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

def load_csv(address):
    # df = pd.read_csv(address, header = 1, names=['A', 'B', 'C', 'D'])
    df = pd.read_csv(address)
    # print(df.iloc[5:,1])
    a = list(df.iloc[5:,1])

    # print(a)
    return a
def load_csv_displacement(address):
    df = pd.read_csv(address)
    a = list(df.iloc[:, 0])
    # print(a)
    return a
def remove_zero(c):
    d = []
    c = [float(item) for item in c]
    for i, item in enumerate(c):
        if item == -0.03:
            d.append(i)
    c = np.delete(c, d)
    c = np.insert(c, 0, 0)

    return  c

def center_peak(c, numb):
    data = []
    for item in c:
        value = float(item)
        if value<0:
            value = 0
        data.append(value)
    peak_id = np.argmax(data)
    print(peak_id)
    while data[peak_id+1]>=data[peak_id]:
        peak_id+=1
    print(peak_id)
    c2 = data[peak_id-numb:peak_id+numb]
    return  c2

def get_time(address):
    df = pd.read_csv(address)
    # print(df.iloc[5:,1])
    worldtime_list = list(df.iloc[1:, 3])
    # worldtime_list = [item.split(" ")[4].split(":") for item in worldtime]
    # print(worldtime)
    # wt_in_sec_list = []
    # for item in worldtime_list:
    #     dt = datetime(2018, 10, 22, int(item[0]), int(item[1]), int(item[2]))
    #     epoch_time = datetime(1970, 1, 1)
    #     delta = (dt - epoch_time).total_seconds()
    #     wt_in_sec_list.append(delta)
    # print(wt_in_sec_list)
    total_time = (worldtime_list[-1]-worldtime_list[0])*0.5*0.001
    print(total_time)
    return  total_time
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    address = 'plane_80kpa_force_test3.csv'
    address_dis = 'plane_80kpa_distance_test3.csv'

    time_cost= get_time(address_dis)
    # c = load_csv(address)

    distance = 10
    speed = 10/60
    # time_cost = int(distance * (1/speed))
    force_sensor_rate = 10

    # c = remove_zero(c)
    # f = np.linspace(0, distance, int(time_cost*force_sensor_rate))
    # plt.plot(f, c[:int(time_cost*force_sensor_rate)], label="test", color = "blue")
    # f = np.linspace(distance, 0, int(time_cost*force_sensor_rate))
    # plt.plot(f, c[int(time_cost*force_sensor_rate):int(time_cost*force_sensor_rate)*2], label="test", color = "blue")

    c = load_csv(address)
    c = center_peak(c, numb=int(time_cost*force_sensor_rate))
    f = np.linspace(0, distance, int(time_cost*force_sensor_rate))
    plt.plot(f, c[:int(time_cost*force_sensor_rate)], label="test", color = "r")
    f = np.linspace(distance, 0, int(time_cost*force_sensor_rate))
    plt.plot(f, c[int(time_cost*force_sensor_rate):int(time_cost*force_sensor_rate)*2], label="test", color = "r")

    address2 = 'plane_80kpa_force_test4.csv'
    address_dis = 'plane_80kpa_distance_test4.csv'


    time_cost= get_time(address_dis)
    c2 = load_csv(address2)
    c2 = center_peak(c2, numb=int(time_cost*force_sensor_rate))
    f = np.linspace(0, distance, int(time_cost*force_sensor_rate))
    plt.plot(f, c2[:int(time_cost*force_sensor_rate)], label="test", color = "b")
    f = np.linspace(distance, 0, int(time_cost*force_sensor_rate))
    plt.plot(f, c2[int(time_cost*force_sensor_rate):int(time_cost*force_sensor_rate)*2], label="test", color = "b")

    # address2 = 'plane_40kpa_force_test2.csv'
    # address_dis = 'plane_40kpa_distance_test2.csv'
    #
    # time_cost = get_time(address_dis)
    # c2 = load_csv(address2)
    # c2 = center_peak(c2, numb=int(time_cost * force_sensor_rate))
    # f = np.linspace(0, distance, int(time_cost * force_sensor_rate))
    # plt.plot(f, c2[:int(time_cost * force_sensor_rate)], label="test", color="y")
    # f = np.linspace(distance, 0, int(time_cost * force_sensor_rate))
    # plt.plot(f, c2[int(time_cost * force_sensor_rate):int(time_cost * force_sensor_rate) * 2], label="test", color="y")
    # #
    # # dis = load_csv_displacement(address_dis)
    # print(c)
    plt.show()
