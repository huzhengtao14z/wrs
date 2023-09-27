import math

import trimesh

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
import hupackage.hupackage as hu
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
    def __init__(self, position_matrix, defaut_dis = 0.005):
        self.position_matrix = position_matrix
        self.wid_num = position_matrix.shape[0]
        self.len_num = position_matrix.shape[1]
        self.defaut_dis = defaut_dis
        self.show()
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
        elif x<0 and y<0:
            return self.position_matrix[0][0]+np.array([-self.defaut_dis, -self.defaut_dis, 0])
    def get_wid(self):
        return self.wid_num
    def get_len(self):
        return self.len_num
    def show(self, thickness = 0.0003):
        for y in range(self.wid_num):
            for x in range(self.len_num):
                gm.gen_sphere(pos = self.position_matrix[y][x], radius = 0.0006, rgba=(0,0,0,1)).attach_to(base)
                if x == self.len_num - 1 and y < self.wid_num - 1:
                    print("check")
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y + 1],
                                 thickness=thickness).attach_to(base)
                elif y == self.wid_num - 1 and x < self.len_num-1:
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x + 1][y],
                                 thickness=thickness).attach_to(base)
                elif x < self.len_num-1 and y < self.wid_num - 1:
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x+1][y], thickness=thickness).attach_to(
                        base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y+1], thickness=thickness).attach_to(
                        base)

        # for y in range(self.wid_num):
        #     gm.gen_stick(self.position_matrix[y][0], self.position_matrix[y][-1], thickness = thickness).attach_to(base)
        # for x in range(self.len_num):
        #     gm.gen_stick(self.position_matrix[0][x], self.position_matrix[-1][x], thickness = thickness).attach_to(base)

class Node(object):
    def __init__(self, grid):
        self.grid = grid
        self.origin_pos_dict = {}
        self.node_infos = {}
        self.node_matrix_infos = {}
        self.origin_offset = 0.001
        self.height = self.grid.defaut_dis
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
                # if self.grid.get_origin(x, y)-(self.grid.get_origin(x + 1, y)+self.grid.get_origin(x - 1, y))/2 == 0:
                #     id_infos["height"] = self.grid.get_origin(x, y)
                #     id_infos["low"] = self.grid.get_origin(x, y)
                # else:
                # print(normal_from_3point([0,0,0],[1,0,0],[1,1,0]))
                # face = trimesh.Trimesh(self.grid.get_origin(x - 1, y),self.grid.get_origin(x + 1, y),self.grid.get_origin(x - 1, y+1))
                normal = hu.normal_from_3point(self.grid.get_origin(x - 1, y),self.grid.get_origin(x + 1, y),self.grid.get_origin(x - 1, y-1))
                print("normal", normal)
                id_infos["height"] = self.grid.get_origin(x, y)+normal*self.height
                # id_infos["height"] = self.grid.get_origin(x, y) - rm.unit_vector(self.grid.get_origin(x + 1, y) - self.grid.get_origin(x - 1, y)) * self.height
                id_infos["low"] = self.grid.get_origin(x, y)-normal*self.height
                id_infos["parity"] = parity
                self.node_infos[f"{x}-{y}"]=id_infos
        self.generate_matrix()
    def generate_matrix(self):
        for id in self.node_infos.keys():
            matrix_id_infos = {}
            matrix_id_infos["parity"] = self.node_infos[id]["parity"]
            if self.node_infos[id]["parity"] == "even-even":
                # matrix_id_infos["top"] = self.node_infos[id]["rgt"] + np.array([-self.origin_offset, 0, 0])
                # matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + np.array([self.origin_offset, 0, 0])
                matrix_id_infos["top"] =  self.node_infos[id]["rgt"] + rm.unit_vector(self.node_infos[id]["origin"]- self.node_infos[id]["rgt"]) * self.origin_offset
                matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + rm.unit_vector(self.node_infos[id]["origin"]- self.node_infos[id]["lft"]) * self.origin_offset
                matrix_id_infos["center1"] = self.node_infos[id]["up"] + np.array([0, -self.origin_offset, 0])
                matrix_id_infos["center2"] = self.node_infos[id]["origin"] + np.array([0, 0, self.height-self.origin_offset])
                matrix_id_infos["center3"] = self.node_infos[id]["down"] + np.array([0, self.origin_offset, 0])
                matrix_id_infos["center4"] = self.node_infos[id]["origin"] + np.array([0, 0, -self.height+self.origin_offset])

                matrix_id_infos["center1"] = self.node_infos[id]["up"] + rm.unit_vector(self.node_infos[id]["origin"]- self.node_infos[id]["up"]) * self.origin_offset
                matrix_id_infos["center2"] = self.node_infos[id]["height"] + rm.unit_vector(self.node_infos[id]["origin"]- self.node_infos[id]["height"]) * self.origin_offset
                matrix_id_infos["center3"] = self.node_infos[id]["down"] + rm.unit_vector(self.node_infos[id]["origin"]- self.node_infos[id]["down"]) * self.origin_offset
                matrix_id_infos["center4"] = self.node_infos[id]["low"] + rm.unit_vector(self.node_infos[id]["origin"]- self.node_infos[id]["low"]) * self.origin_offset
            elif self.node_infos[id]["parity"] == "odd-even":
                # matrix_id_infos["top"] = self.node_infos[id]["rgt"] + np.array([-self.origin_offset, 0, 0])
                # matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + np.array([self.origin_offset, 0, 0])
                matrix_id_infos["top"] = self.node_infos[id]["rgt"] + rm.unit_vector(self.node_infos[id]["origin"] - self.node_infos[id]["rgt"]) * self.origin_offset
                matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + rm.unit_vector(self.node_infos[id]["origin"] - self.node_infos[id]["lft"]) * self.origin_offset
                # matrix_id_infos["center1"] = self.node_infos[id]["up"] + np.array([0, -(0.005-self.origin_offset)/1.414, (0.005-self.origin_offset)/1.414])
                # matrix_id_infos["center2"] = self.node_infos[id]["origin"] + np.array([0, -(0.005-self.origin_offset)/1.414, (0.005-self.origin_offset)/1.414])
                # matrix_id_infos["center3"] = self.node_infos[id]["down"] + np.array([0, (0.005-self.origin_offset)/1.414, -(0.005-self.origin_offset)/1.414])
                # matrix_id_infos["center4"] = self.node_infos[id]["origin"] + np.array([0, (0.005-self.origin_offset)/1.414, -(0.005-self.origin_offset)/1.414])

                matrix_id_infos["center1"] = self.node_infos[id]["origin"] + (
                            self.node_infos[id]["up"] - self.node_infos[id]["origin"] + np.array(
                        [0, -self.origin_offset, self.height - self.origin_offset])) / 1.414
                matrix_id_infos["center2"] = self.node_infos[id]["origin"] + (
                            self.node_infos[id]["down"] - self.node_infos[id]["origin"] + np.array(
                        [0, self.origin_offset, self.height - self.origin_offset])) / 1.414
                matrix_id_infos["center3"] = self.node_infos[id]["origin"] + (
                            self.node_infos[id]["down"] - self.node_infos[id]["origin"] + np.array(
                        [0, self.origin_offset, -self.height + self.origin_offset])) / 1.414
                matrix_id_infos["center4"] = self.node_infos[id]["origin"] + (
                            self.node_infos[id]["up"] - self.node_infos[id]["origin"] + np.array(
                        [0, -self.origin_offset, -self.height + self.origin_offset])) / 1.414


            elif self.node_infos[id]["parity"] == "even-odd":
                # matrix_id_infos["top"] = self.node_infos[id]["up"] + np.array([0, -self.origin_offset, 0])
                # matrix_id_infos["bottom"] = self.node_infos[id]["down"] + np.array([0, self.origin_offset, 0])
                matrix_id_infos["top"] = self.node_infos[id]["up"] + rm.unit_vector(
                    self.node_infos[id]["origin"] - self.node_infos[id]["up"]) * self.origin_offset
                matrix_id_infos["bottom"] = self.node_infos[id]["down"] + rm.unit_vector(
                    self.node_infos[id]["origin"] - self.node_infos[id]["down"]) * self.origin_offset
                matrix_id_infos["center1"] = self.node_infos[id]["origin"] + (self.node_infos[id]["rgt"] - self.node_infos[id]["origin"] + np.array([-self.origin_offset, 0, self.height-self.origin_offset]))/ 1.414
                matrix_id_infos["center2"] = self.node_infos[id]["origin"] + (self.node_infos[id]["lft"] - self.node_infos[id]["origin"] + np.array([self.origin_offset, 0, self.height -self.origin_offset])) / 1.414
                matrix_id_infos["center3"] = self.node_infos[id]["origin"] + (self.node_infos[id]["lft"] - self.node_infos[id]["origin"] + np.array([self.origin_offset, 0, -self.height +self.origin_offset])) / 1.414
                matrix_id_infos["center4"] = self.node_infos[id]["origin"] + (self.node_infos[id]["rgt"] - self.node_infos[id]["origin"] + np.array([-self.origin_offset, 0, -self.height + self.origin_offset])) / 1.414

                # matrix_id_infos["center1"] = self.node_infos[id]["origin"] + np.array([-(0.005-self.origin_offset)/1.414, 0, (0.005-self.origin_offset)/1.414])
                # matrix_id_infos["center2"] = self.node_infos[id]["origin"] + np.array([-(0.005-self.origin_offset)/1.414, 0, -(0.005-self.origin_offset)/1.414])
                # matrix_id_infos["center3"] = self.node_infos[id]["origin"] + np.array([(0.005-self.origin_offset)/1.414, 0, -(0.005-self.origin_offset)/1.414])
                # matrix_id_infos["center4"] = self.node_infos[id]["origin"] + np.array([(0.005-self.origin_offset)/1.414, 0, (0.005-self.origin_offset)/1.414])
            else:
                pass

            self.node_matrix_infos[id] = matrix_id_infos


class Element(object):
    def __init__(self, node, radius = 0.0002):
        # self.construct()
        self.radius = radius
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
        t_c1 = capsule_link_start_end(self.t, self.c1, self.radius)
        t_c2 = capsule_link_start_end(self.t, self.c2, self.radius)
        t_c3 = capsule_link_start_end(self.t, self.c3, self.radius)
        t_c4 = capsule_link_start_end(self.t, self.c4, self.radius)
        b_c1 = capsule_link_start_end(self.b, self.c1, self.radius)
        b_c2 = capsule_link_start_end(self.b, self.c2, self.radius)
        b_c3 = capsule_link_start_end(self.b, self.c3, self.radius)
        b_c4 = capsule_link_start_end(self.b, self.c4, self.radius)
        c1_c2 = capsule_link_start_end(self.c1, self.c2, self.radius)
        c2_c3 = capsule_link_start_end(self.c2, self.c3, self.radius)
        c3_c4 = capsule_link_start_end(self.c3, self.c4, self.radius)
        c4_c1 = capsule_link_start_end(self.c4, self.c1, self.radius)
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
    # base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960,
    #                 h=540, lookat_pos=[0, 0, 0.0])
    base = wd.World(cam_pos=[0, 0, 0.2], w=960,
                    h=540, lookat_pos=[0, 0, 0.0])
    # gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)

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
    len_num = 5
    wid_num = 5
    #----------------------
    #even matrix
    # matrix = [[np.array([interval*x, interval*y, 0.000]) for x in range(len_num)] for y in range(wid_num)]
    #----------------------
    # uneven matrix
    matrix = []
    for y in range(wid_num):
        temp = []
        for x in range(len_num):
            para_x = (x-5)/5
            para_y = (y - 5) / 5
            temp.append(np.array([interval * x + x*para_x*0.002, interval * y + y*para_y*0.002, 0.000]))
        matrix.append(temp)
    # ----------------------
    #place curve matrix
    # matrix = [[np.array([interval * x, interval * y, 0.000]) for x in range(len_num)] for y in range(wid_num)]
    # for y in range(len_num):
    #     for x in range(wid_num):
    #         if x ==0 and y ==0:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.010, 0, 0.000])
    #         elif x ==1 and y ==0:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.006, 0, 0.000])
    #         elif x ==3 and y ==0:
    #             matrix[y][x] = matrix[y][x]+np.array([0.006, 0, 0.000])
    #         elif x ==4 and y ==0:
    #             matrix[y][x] = matrix[y][x]+np.array([0.010, 0, 0.000])
    #         elif x ==0 and y ==1:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.004, 0, 0.000])
    #         elif x ==1 and y ==1:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.002, 0, 0.000])
    #         elif x ==3 and y ==1:
    #             matrix[y][x] = matrix[y][x]+np.array([0.002, 0, 0.000])
    #         elif x ==4 and y ==1:
    #             matrix[y][x] = matrix[y][x]+np.array([0.004, 0, 0.000])
    #         elif x ==0 and y ==3:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.004, 0, 0.000])
    #         elif x ==1 and y ==3:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.002, 0, 0.000])
    #         elif x ==3 and y ==3:
    #             matrix[y][x] = matrix[y][x]+np.array([+0.002, 0, 0.000])
    #         elif x ==4 and y ==3:
    #             matrix[y][x] = matrix[y][x]+np.array([+0.004, 0, 0.000])
    #         elif x ==0 and y ==4:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.010, 0, 0.000])
    #         elif x ==1 and y ==4:
    #             matrix[y][x] = matrix[y][x]+np.array([-0.006, 0, 0.000])
    #         elif x ==3 and y ==4:
    #             matrix[y][x] = matrix[y][x]+np.array([+0.006, 0, 0.000])
    #         elif x ==4 and y ==4:
    #             matrix[y][x] = matrix[y][x]+np.array([+0.010, 0, 0.000])

    # c1 = cm.gen_box(extent=[.010, .050, .001], rgba=[0,0,0,0.2]).attach_to(base)
    # c2 = cm.gen_box(extent=[.010, .050, .001], homomat=rm.homomat_from_posrot([0,0,0], rm.rotmat_from_axangle([0,0,1], np.pi/2)),rgba=[0, 0, 0, 0.2]).attach_to(base)
    # c3 = cm.gen_box(extent=[.010, .050, .001], homomat=rm.homomat_from_posrot([0, 0.02, 0], rm.rotmat_from_axangle([0, 0, 1], np.pi / 2)),
    #            rgba=[0, 0, 0, 0.2]).attach_to(base)
    # c4 = cm.gen_box(extent=[.010, .050, .001], homomat=rm.homomat_from_posrot([0.02,0,0], rm.rotmat_from_axangle([0,0,1], 0)), rgba=[0, 0, 0, 0.2]).attach_to(base)
    grid = Grid(np.array(matrix), interval)
    node = Node(grid)
    matrix_infos = node.node_matrix_infos
    for key in matrix_infos.keys():
        if key == "2-2":
            element = Element(matrix_infos[key], radius=0.0002)




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