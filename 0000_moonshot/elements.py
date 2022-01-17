import math
import visualization.panda.world as wd
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.yumi_gripper.yumi_gripper as yg
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as hnde
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode
import numpy as np
import basis.robot_math as rm
import modeling.geometric_model as gm
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_sim.robots.ur3e_dual.ur3e_dual as ur3ed
import robot_sim.robots.sda5f.sda5f as sda5
import motion.probabilistic.rrt_connect as rrtc
import robot_sim._kinematics.collision_checker as cc
import robot_sim.end_effectors.grippers.gripper_interface as gi
# import robot_sim.
def capsule_link_start_end(start, end, radius = 0.0003):
    start = start
    end = end
    radius = radius
    cylinder = cm.gen_capsule(spos=start, epos=end, radius=radius, section=[5, 5])
    return cylinder

class Grid(object):
    def __init__(self, position_matrix):
        self.position_matrix = position_matrix
        self.wid_num = position_matrix.shape[0]
        self.len_num = position_matrix.shape[1]
        print(self.len_num, self.wid_num)
        self.defaut_dis = 0.005
    def get_origin(self, x, y):
        if 0<= x < self.len_num and 0<= y < self.wid_num:
            # print(x,y)
            return self.position_matrix[y][x]
        elif x<0 and 0<= y < self.wid_num:
            return self.position_matrix[y][0]+np.array([-self.defaut_dis, 0, 0])
        elif x>=self.len_num and 0<= y < self.wid_num:
            return self.position_matrix[y][x-1]+np.array([self.defaut_dis, 0, 0])
        elif y<0 and 0<= x < self.len_num:
            return self.position_matrix[0][x]+np.array([0, -self.defaut_dis, 0])
        elif y>=self.wid_num and 0<= x < self.len_num:
            return self.position_matrix[y-1][x]+np.array([0, self.defaut_dis, 0])
    def get_wid(self):
        return self.wid_num
    def get_len(self):
        return self.len_num

class Element(object):
    def __init__(self, node):
        self.t = node.get_position_top()
        self.b = node.get_position_bottom()
        self.c1 = node.get_position_c1()
        self.c2 = node.get_position_c2()
        self.c3 = node.get_position_c3()
        self.c4 = node.get_position_c4()
        if node.space == False:
            self.construct()
        else:
            pass

    def construct(self):
        t_c1 = capsule_link_start_end(self.t, self.c1)
        t_c2 = capsule_link_start_end(self.t, self.c2)
        t_c3 = capsule_link_start_end(self.t, self.c3)
        t_c4 = capsule_link_start_end(self.t, self.c4)
        b_c1 = capsule_link_start_end(self.b, self.c1)
        b_c2 = capsule_link_start_end(self.b, self.c2)
        b_c3 = capsule_link_start_end(self.b, self.c3)
        b_c4 = capsule_link_start_end(self.b, self.c4)
        c1_c2 = capsule_link_start_end(self.c1, self.c2)
        c2_c3 = capsule_link_start_end(self.c2, self.c3)
        c3_c4 = capsule_link_start_end(self.c3, self.c4)
        c4_c1 = capsule_link_start_end(self.c4, self.c1)
        t_c1.attach_to(base)
        t_c2.attach_to(base)
        t_c3.attach_to(base)
        t_c4.attach_to(base)
        b_c1.attach_to(base)
        b_c2.attach_to(base)
        b_c3.attach_to(base)
        b_c4.attach_to(base)
        c1_c2.attach_to(base)
        c2_c3.attach_to(base)
        c3_c4.attach_to(base)
        c4_c1.attach_to(base)
class Node(object):
    def __init__(self, origin = None, parity = "even"):
        self.origin = origin
        self.parity = parity
        self.space = False
        # self.top = position_dict["top"]
        # self.bottom= position_dict["bottom"]
        # self.center1 = position_dict["center1"]
        # self.center2 = position_dict["center2"]
        # self.center3 = position_dict["center3"]
        # self.center4 = position_dict["center4"]
        if self.parity == "even":
            self.distance_x1 = np.array([0.004, 0, 0])
            self.distance_1x = np.array([-0.004, 0, 0])
            self.distance_y1 = np.array([0, 0.004, 0])
            self.distance_1y = np.array([0, -0.004, 0])
            self.distance_z1 = np.array([0, 0, 0.004])
            self.distance_1z = np.array([0, 0, -0.004])
            self.generate_matrix()
        elif self.parity == "odd":
            self.distance_x1 = np.array([0.004, 0, 0])
            self.distance_1x = np.array([-0.004, 0, 0])
            self.distance_y1z1 = np.array([0, 0.004/1.414, 0.004/1.414])
            self.distance_y11z = np.array([0, 0.004/1.414, -0.004/1.414])
            self.distance_1yz1 = np.array([0, -0.004/1.414, 0.004/1.414])
            self.distance_1y1z = np.array([0, -0.004/1.414, -0.004/1.414])
            self.generate_matrix()
        else:
            self.space = True

    def generate_matrix(self):
        self.top = self.origin + self.distance_x1
        self.bottom = self.origin + self.distance_1x
        if self.parity == "even":
            self.center1 = self.origin + self.distance_y1
            self.center2 = self.origin + self.distance_z1
            self.center3 = self.origin + self.distance_1y
            self.center4 = self.origin + self.distance_1z
        else:
            self.center1 = self.origin + self.distance_y1z1
            self.center2 = self.origin + self.distance_y11z
            self.center3 = self.origin + self.distance_1y1z
            self.center4 = self.origin + self.distance_1yz1

    def get_position_top(self):
        return self.top

    def get_position_bottom(self):
        return self.bottom

    def get_position_c1(self):
        return self.center1

    def get_position_c2(self):
        return self.center2

    def get_position_c3(self):
        return self.center3

    def get_position_c4(self):
        return self.center4

class Node2(object):
    def __init__(self, grid):
        self.grid = grid
        self.origin_pos_dict = {}
        self.node_infos = {}
        self.node_matrix_infos = {}
        self.origin_offset = 0.001
        for y in range(self.grid.get_wid()):
            for x in range(self.grid.get_len()):
                #id = f"{x}-{y}"
                if y % 2 == 0 and x % 2 != 0:
                    parity = "odd-even"
                elif y % 2 != 0 and x % 2 != 0:
                    parity = "space"
                elif y % 2 != 0 and x % 2 == 0:
                    parity = "even-odd"
                else:
                    parity = "even-even"
                id_infos = {}
                id_infos["origin"] = self.grid.get_origin(x, y)
                id_infos["up"] = self.grid.get_origin(x , y+ 1)
                id_infos["down"] = self.grid.get_origin(x , y- 1)
                id_infos["lft"] = self.grid.get_origin(x- 1, y )
                id_infos["rgt"] = self.grid.get_origin(x+ 1, y )
                id_infos["parity"] = parity
                self.node_infos[f"{x}-{y}"]=id_infos
        self.generate_matrix()
    def generate_matrix(self):
        for id in self.node_infos.keys():
            matrix_id_infos = {}
            matrix_id_infos["parity"] = self.node_infos[id]["parity"]
            if self.node_infos[id]["parity"] == "even-even":
                matrix_id_infos["top"] = self.node_infos[id]["rgt"] + np.array([-self.origin_offset, 0, 0])
                matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + np.array([self.origin_offset, 0, 0])
                matrix_id_infos["center1"] = self.node_infos[id]["up"] + np.array([0, -self.origin_offset, 0])
                matrix_id_infos["center2"] = self.node_infos[id]["origin"] + np.array([0, 0, 0.005-self.origin_offset])
                matrix_id_infos["center3"] = self.node_infos[id]["down"] + np.array([0, self.origin_offset, 0])
                matrix_id_infos["center4"] = self.node_infos[id]["origin"] + np.array([0, 0, -0.005+self.origin_offset])
            elif self.node_infos[id]["parity"] == "odd-even":
                matrix_id_infos["top"] = self.node_infos[id]["rgt"] + np.array([-self.origin_offset, 0, 0])
                matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + np.array([self.origin_offset, 0, 0])
                matrix_id_infos["center1"] = self.node_infos[id]["up"] + np.array([0, -(0.005-self.origin_offset)/1.414, (0.005-self.origin_offset)/1.414])
                matrix_id_infos["center2"] = self.node_infos[id]["origin"] + np.array([0, -(0.005-self.origin_offset)/1.414, (0.005-self.origin_offset)/1.414])
                matrix_id_infos["center3"] = self.node_infos[id]["down"] + np.array([0, (0.005-self.origin_offset)/1.414, -(0.005-self.origin_offset)/1.414])
                matrix_id_infos["center4"] = self.node_infos[id]["origin"] + np.array([0, (0.005-self.origin_offset)/1.414, -(0.005-self.origin_offset)/1.414])
            elif self.node_infos[id]["parity"] == "even-odd":
                matrix_id_infos["top"] = self.node_infos[id]["up"] + np.array([0, -self.origin_offset, 0])
                matrix_id_infos["bottom"] = self.node_infos[id]["down"] + np.array([0, self.origin_offset, 0])
                matrix_id_infos["center1"] = self.node_infos[id]["origin"] + np.array([-(0.005-self.origin_offset)/1.414, 0, (0.005-self.origin_offset)/1.414])
                matrix_id_infos["center2"] = self.node_infos[id]["origin"] + np.array([-(0.005-self.origin_offset)/1.414, 0, -(0.005-self.origin_offset)/1.414])
                matrix_id_infos["center3"] = self.node_infos[id]["origin"] + np.array([(0.005-self.origin_offset)/1.414, 0, -(0.005-self.origin_offset)/1.414])
                matrix_id_infos["center4"] = self.node_infos[id]["origin"] + np.array([(0.005-self.origin_offset)/1.414, 0, (0.005-self.origin_offset)/1.414])
            else:
                pass

            self.node_matrix_infos[id] = matrix_id_infos


class Element2(object):
    def __init__(self, node):
        # self.construct()
        if node["parity"] == "space":
            pass
        else:
            self.t = node["top"]
            self.b = node["bottom"]
            self.c1 = node["center1"]
            self.c2 = node["center2"]
            self.c3 = node["center3"]
            self.c4 = node["center4"]
            self.construct()

    def construct(self):
        t_c1 = capsule_link_start_end(self.t, self.c1)
        t_c2 = capsule_link_start_end(self.t, self.c2)
        t_c3 = capsule_link_start_end(self.t, self.c3)
        t_c4 = capsule_link_start_end(self.t, self.c4)
        b_c1 = capsule_link_start_end(self.b, self.c1)
        b_c2 = capsule_link_start_end(self.b, self.c2)
        b_c3 = capsule_link_start_end(self.b, self.c3)
        b_c4 = capsule_link_start_end(self.b, self.c4)
        c1_c2 = capsule_link_start_end(self.c1, self.c2)
        c2_c3 = capsule_link_start_end(self.c2, self.c3)
        c3_c4 = capsule_link_start_end(self.c3, self.c4)
        c4_c1 = capsule_link_start_end(self.c4, self.c1)
        t_c1.attach_to(base)
        t_c2.attach_to(base)
        t_c3.attach_to(base)
        t_c4.attach_to(base)
        b_c1.attach_to(base)
        b_c2.attach_to(base)
        b_c3.attach_to(base)
        b_c4.attach_to(base)
        c1_c2.attach_to(base)
        c2_c3.attach_to(base)
        c3_c4.attach_to(base)
        c4_c1.attach_to(base)
if __name__ == '__main__':
    base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960,
                    h=540, lookat_pos=[0, 0, 0.0])
    gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)

    # position_dict_00 = {"top": np.array([ 0.004,  0.000,  0.000]),
    #                  "bottom": np.array([-0.004,  0.000,  0.000]),
    #                 "center1": np.array([ 0.000,  0.004,  0.000]),
    #                 "center2": np.array([ 0.000,  0.000,  0.004]),
    #                 "center3": np.array([ 0.000, -0.004,  0.000]),
    #                 "center4": np.array([ 0.000,  0.000, -0.004]),
    #                 }
    # position_dict_01 = {"top": np.array([0.009, 0.000, 0.000]),
    #                     "bottom": np.array([0.001, 0.000, 0.000]),
    #                     "center1": np.array([0.005, 0.004/1.414, 0.004/1.414]),
    #                     "center2": np.array([0.005, 0.004/1.414,-0.004/1.414]),
    #                     "center3": np.array([0.005, -0.004/1.414,-0.004/1.414]),
    #                     "center4": np.array([0.005, -0.004/1.414, 0.004/1.414]),
    #                     }
    interval = 0.005
    len_num = 10
    wid_num = 15
    matrix = [[np.array([interval*x, interval*y, 0.000]) for x in range(len_num)] for y in range(wid_num)]
    grid = Grid(np.array(matrix))
    node2 = Node2(grid)
    matrix_infos = node2.node_matrix_infos
    for key in matrix_infos.keys():
        element = Element2(matrix_infos[key])




    # print(matrix)
    # node_origin_00 = np.array([0.000, 0.000, 0.000])
    # node_origin_10 = np.array([0.005, 0.000, 0.000])
    # node_origin_20 = np.array([0.010, 0.000, 0.000])
    # node_origin_30 = np.array([0.015, 0.000, 0.000])
    # node_origin_40 = np.array([0.020, 0.000, 0.000])
    # node_origin_50 = np.array([0.025, 0.000, 0.000])
    # node_origin_60 = np.array([0.030, 0.000, 0.000])
    # node_origin_70 = np.array([0.035, 0.000, 0.000])
    # node_00 = Node(node_origin_00, "even")
    # node_10 = Node(node_origin_10, "odd")
    # node_20 = Node(node_origin_20, "even")
    # node_30 = Node(node_origin_30, "odd")
    # node_40 = Node(node_origin_40, "even")
    # node_50 = Node(node_origin_50, "odd")
    # node_60 = Node(node_origin_60, "even")
    # node_70 = Node(node_origin_70, "odd")
    # element_00 = Element(node_00)
    # element_10 = Element(node_10)
    # element_20 = Element(node_20)
    # element_30 = Element(node_30)
    # element_40 = Element(node_40)
    # element_50 = Element(node_50)
    # element_60 = Element(node_60)
    # element_70 = Element(node_70)





    def update(textNode, task):
        if textNode[0] is not None:
            textNode[0].detachNode()
            textNode[1].detachNode()
            textNode[2].detachNode()
        cam_pos = base.cam.getPos()
        textNode[0] = OnscreenText(
            text=str(cam_pos[0])[0:5],
            fg=(1, 0, 0, 1),
            pos=(1.0, 0.8),
            align=TextNode.ALeft)
        textNode[1] = OnscreenText(
            text=str(cam_pos[1])[0:5],
            fg=(0, 1, 0, 1),
            pos=(1.3, 0.8),
            align=TextNode.ALeft)
        textNode[2] = OnscreenText(
            text=str(cam_pos[2])[0:5],
            fg=(0, 0, 1, 1),
            pos=(1.6, 0.8),
            align=TextNode.ALeft)
        return task.again


    cam_view_text = OnscreenText(
        text="Camera View: ",
        fg=(0, 0, 0, 1),
        pos=(1.15, 0.9),
        align=TextNode.ALeft)
    testNode = [None, None, None]
    taskMgr.add(update, "addobject", extraArgs=[testNode], appendTask=True)
    base.run()