import math
import pickle
import re
import time
import os
import config
from socket import socket, AF_INET, SOCK_STREAM

import matplotlib.pyplot as plt
from direct.stdpy import threading


class UR3eServer(object):
    def __init__(self, host, port, backlog=5, buffer_size=8192):
        self.host = host
        self.port = port
        self.backlog = backlog
        self.buffer_size = buffer_size
        self.exp_name = "wrist"
        self.exp_id = "mtp"
        self.__socket = socket(AF_INET, SOCK_STREAM)
        self.__socket.bind((self.host, self.port))
        self.__socket.listen(self.backlog)
        self.__file = "ftdata/test"
        self.__gripper = []

        self.__current_index = 0
        try:
            print("in")
            self.__state = pickle.load(
                open(os.path.join(config.ROOT, f"{self.__file}", f"{self.exp_name}_state.pkl"), "rb"))
            self.__state = [int(s) for s in self.__state]
            info = pickle.load(open(f"./{self.__file}/{self.exp_name + '_' + self.exp_id}.pkl", "rb"))
            self.__force = info[0]
            self.__armjnts = info[1]
            self.__speed = info[2]
            self.__diff = info[3]
            self.__tcppose = info[4]
            self.__gripper = info[5]
        except:
            self.__state = []
            self.__force = []
            self.__armjnts = []
            self.__speed = []
            self.__diff = []
            self.__tcppose = []
            self.__gripper = []
            self.__status = []

        self.__flag = True
        self.__ploton = True
        self.__plotflag = False

    def start(self):
        def plot():
            print("plot start")
            fig = plt.figure(1, figsize=(16, 9))
            plt.ion()
            plt.show()
            plt.ylim((-3, 3))
            while self.__ploton:
                if self.__plotflag:
                    plt.clf()
                    force = [l[:3] for l in self.__force]
                    torque = [l[3:] for l in self.__force]
                    torque_onz = [math.sqrt((l[4]*l[4])+(l[5]*l[5])) for l in self.__force]
                    speed_l = [l[:3] for l in self.__speed]
                    speed_r = [l[3:] for l in self.__speed]
                    tcprot = [l[3:] for l in self.__tcppose]
                    gripperdistance = self.__gripper

                    diff = [d for d in self.__diff]
                    if len(diff) != len(force) or len(diff) != len(speed_l) or len(diff) != len(tcprot) or len(
                            force) != len(speed_l) or len(force) != len(tcprot) or len(speed_l) != len(tcprot):
                        continue
                    x = [i for i in range(len(force))]
                    # plt.xlim((max(x)-10, max(x)))
                    # plt.subplot(231)
                    # plt.xlim((max(x) - 10, max(x)))
                    # plt.plot(x, force, label=["Fx", "Fy", "Fz"])
                    # plt.title("Force")
                    # plt.legend(("Fx", "Fy", "Fz"), loc='upper left')
                    #
                    plt.subplot(211)
                    # plt.xlim((max(x) - 20, max(x)))
                    # plt.plot(x, torque, label=["Rx", "Ry", "Rz"])
                    # plt.title("Torque")
                    # plt.legend(("Rx", "Ry", "Rz"), loc='upper left')

                    plt.plot(x, torque_onz)
                    plt.title("Torque")
                    # plt.legend(("Rx", "Ry", "Rz"), loc='upper left')

                    plt.subplot(212)
                    # plt.xlim((max(x) - 20, max(x)))
                    # plt.plot(x, gripperdistance[1:], label=["distance"])
                    # plt.title("distance")
                    # plt.legend(("Rx", "Ry", "Rz"), loc='upper left')
                    #
                    # plt.subplot(233)
                    # plt.xlim((max(x) - 10, max(x)))
                    # plt.plot(x, diff, label="diff")
                    # plt.title("TCP deviation")
                    # plt.legend(loc='upper left')

                    # plt.subplot(234)
                    # plt.plot(x, speed_l, label=["x", "y", "z"])
                    # plt.title("Speed")
                    # plt.legend(("x", "y", "z"), loc='upper left')
                    #
                    # plt.subplot(235)
                    # plt.plot(x, speed_r, label=["Rx", "Ry", "Rz"])
                    # plt.title("Speed")
                    # plt.legend(("Rx", "Ry", "Rz"), loc='upper left')
                    #
                    # plt.subplot(236)
                    # plt.legend(("Rx", "Ry", "Rz"), loc='upper left')
                    # plt.title("TCP rotation")
                    # plt.plot(x, tcprot, label=["Rx", "Ry", "Rz"])

                    plt.pause(0.005)
                time.sleep(.1)
            # plt.savefig(f"./{self.__file}/{self.exp_name}_{self.exp_id}.png")
            plt.close(fig)

        self.__thread_plot = threading.Thread(target=plot, name="plot")
        self.__thread_plot.start()
        while self.__flag:
            try:
                conn, address = self.__socket.accept()
                # print('Got connection from {}'.format(address))
                while True:
                    msg = conn.recv(self.buffer_size)
                    if not msg:
                        break
                    msg = re.findall(r"b\'(.+)\'", str(msg))[0]
                    if msg == "stop":
                        print("stop message received!")
                        # self.__stop_msg_handler()
                        conn.close()
                        self.__socket.close()
                        break
                    else:
                        self.__msg_handler(msg)
                        self.__plotflag = True

            except:
                self.__stop_msg_handler()
                self.__socket.close()
                self.__flag = False
                self.__ploton = False
                time.sleep(.05)
                self.__plotflag = False
                self.__thread_plot.join()
                self.__thread_plot = None

    def __stop_msg_handler(self):
        print("robot stop!")
        pickle.dump([self.__force, self.__armjnts, self.__speed, self.__diff, self.__tcppose, self.__gripper],
                    open(f"./{self.__file}/{self.exp_name + '_' + self.exp_id}.pkl", "wb"))
        self.__state.append(self.__current_index + 1)
        pickle.dump(self.__state, open(f"./{self.__file}/{self.exp_name}_state.pkl", "wb"))

    def __msg_handler(self, msg):
        # msg = msg.decode(encoding="ascii")
        print(msg)
        if msg[0] == "f":
            self.__force.append(eval(msg[2:]))
            print("force:", self.__force[-1])
        if msg[0] == "s":
            self.__speed.append(eval(msg[2:]))
            print("speed:", self.__speed[-1])
        if msg[0] == "t":
            self.__tcppose.append(eval(msg[2:]))
            print("tcp rotation:", self.__tcppose[-1])
        if msg[0] == "a":
            self.__armjnts.append([a * 180 / math.pi for a in eval(msg[1:])])
            print("armjnts:", self.__armjnts[-1])
        if msg[0] == "d":
            self.__diff.append(eval(msg[1:]))
            print("diff:", msg[1:])
        if msg[0] == "i":
            self.__current_index = int(msg[1:])
            self.__gripper.append(0)
            print("index:", self.__current_index)

        # if msg[0] == 'h':
        #     self.__status = int(msg[1:])
        #     self.__gripper.append(self.__status)
        #     print("gripperdictance", self.__gripper[-1])
        if msg[0] == 'h':
            self.__status = int(msg[1:])
            # self.__gripper.pop(-1)
            self.__gripper[-1] = self.__status
            print("gripperdictance", self.__gripper[-1])
        print("debug check", len(self.__force), len(self.__gripper))

if __name__ == '__main__':
    ur3e_server = UR3eServer('0.0.0.0', 8000)
    ur3e_server.start()
