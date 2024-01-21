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
import trimesh.boolean as tb
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_sim.robots.ur3e_dual.ur3e_dual as ur3ed
import robot_sim.robots.sda5f.sda5f as sda5
import motion.probabilistic.rrt_connect as rrtc
import robot_sim._kinematics.collision_checker as cc
import robot_sim.end_effectors.grippers.gripper_interface as gi
# import robot_sim.
import os
def capsule_link_start_end(start, end, radius = 0.0003, rgba = (.5,0.5,.5,1)):
    start = start
    end = end
    radius = radius
    capsule = cm.gen_capsule(spos=start, epos=end, radius=radius, section=[30, 30],  rgba = rgba)
    return capsule

def cylinder_link_start_end(start, end,  radius = 0.0003):
    start = start
    end = end
    height = np.linalg.norm(end - start, ord = 2, axis = 0, keepdims=True)[0]
    vector = rm.unit_vector(end - start)
    rot3 = rm.rotmat_between_vectors(np.array([0,0,1]),vector)
    rot4 = rm.homomat_from_posrot(pos = start, rot = rot3)
    cylinder = cm.gen_cylinder(radius=radius, height=height, section=30, homomat=rot4)
    return cylinder


def cpt_vec(vec_a, vec_b, offset):
    result_vec = (vec_a - vec_b - rm.unit_vector(vec_a - vec_b) * offset) / 1.414
    return result_vec
class Grid(object):
    def __init__(self, position_matrix, defaut_dis = 0.005):
        self.position_matrix = position_matrix
        self.wid_num = position_matrix.shape[1]
        self.len_num = position_matrix.shape[0]
        self.defaut_dis = defaut_dis
        self.show()
    def get_origin(self, x, y):
        if 0<= x < self.len_num and 0<= y < self.wid_num:
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
        elif x>self.len_num and y>self.wid_num:
            return self.position_matrix[0][0]+np.array([self.defaut_dis, self.defaut_dis, 0])
    def get_wid(self):
        return self.wid_num
    def get_len(self):
        return self.len_num

    def show(self, thickness = 0.0003):
        r = 0.001
        surface = cm.gen_box(extent=[(self.wid_num-3)* self.defaut_dis, (self.len_num-3)* self.defaut_dis, .00001],
                        homomat=rm.homomat_from_posrot([self.defaut_dis * 4, self.defaut_dis * 4, 0], rm.rotmat_from_axangle([0, 0, 1], 0)),
                        rgba=[241/255, 167/255, 56/255, 0.4]).attach_to(base)


        for y in range(1, self.wid_num-1):
            for x in range(1, self.len_num-1):
                # gm.gen_sphere(pos=self.position_matrix[y][x], radius=0.0005, rgba=(0.7,0.7, 0.7, 1)).attach_to(base)
                def drawnode():
                    if (x % 2) == 0 and (y % 2) == 0:
                        gm.gen_sphere(pos = self.position_matrix[y][x], radius = r, rgba=(0,0,0,1)).attach_to(base)
                    elif (x % 2) == 1 and (y % 2) == 1:
                        gm.gen_sphere(pos = self.position_matrix[y][x], radius = r, rgba=(255/255,65/255,148/255,1)).attach_to(base)
                    elif (x % 2) == 0 and (y % 2) == 1:
                        gm.gen_sphere(pos = self.position_matrix[y][x], radius = r, rgba=(0/255,227/255,121/255,1)).attach_to(base)
                    elif (x % 2) == 1 and (y % 2) == 0:
                        gm.gen_sphere(pos = self.position_matrix[y][x], radius = r, rgba=(245/255,130/255,32/255,1)).attach_to(base)

                if x == self.len_num - 2 and y < self.wid_num - 2:
                    gm.gen_sphere(pos = self.position_matrix[y][x], radius = r, rgba=(0.7,0.7,0.7,1)).attach_to(base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y + 1], rgba=(0.7, 0.7, 0.7, 1),
                                 thickness=thickness).attach_to(base)

                elif y == self.wid_num - 2 and x < self.len_num-2:
                    gm.gen_sphere(pos = self.position_matrix[y][x], radius = r, rgba=(0.7,0.7,0.7,1)).attach_to(base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x + 1][y],rgba=(0.7, 0.7, 0.7, 1),
                                 thickness=thickness).attach_to(base)


                elif y == 1 and  1< x < self.len_num-2:
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x + 1][y],rgba=(0.7, 0.7, 0.7, 1),
                                 thickness=thickness).attach_to(base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y + 1],
                                 thickness=thickness).attach_to(base)
                    gm.gen_sphere(pos=self.position_matrix[y][x], radius=r, rgba=(0.7, 0.7, 0.7, 1)).attach_to(base)

                elif x == 1 and  1<y < self.wid_num - 2:
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x + 1][y],thickness=thickness).attach_to(base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y + 1],rgba=(0.7, 0.7, 0.7, 1),
                                 thickness=thickness).attach_to(base)
                    gm.gen_sphere(pos=self.position_matrix[y][x], radius=r, rgba=(0.7, 0.7, 0.7, 1)).attach_to(base)
                # 80 / 255, 185 / 255, 1, 1
                elif x == 1 and  y==1:
                    gm.gen_sphere(pos=self.position_matrix[y][x], radius=r, rgba=(0.7, 0.7, 0.7, 1)).attach_to(
                        base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x + 1][y],rgba=(0.7, 0.7, 0.7, 1),thickness=thickness).attach_to(base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y + 1],rgba=(0.7, 0.7, 0.7, 1),thickness=thickness).attach_to(base)
                elif x == self.len_num-2 and y == self.wid_num - 2:
                    gm.gen_sphere(pos=self.position_matrix[y][x], radius=r, rgba=(0.7, 0.7, 0.7, 1)).attach_to(
                        base)

                elif x < self.len_num-2 and y < self.wid_num - 2:
                    drawnode()
                    print("x:",x,"====", "y:",y)
                    print(self.position_matrix.shape)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x+1][y], thickness=thickness).attach_to(
                        base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y+1], thickness=thickness).attach_to(
                        base)


        # for y in range(self.wid_num):
        #     gm.gen_stick(self.position_matrix[y][0], self.position_matrix[y][-1], thickness = thickness).attach_to(base)
        # for x in range(self.len_num):
        #     gm.gen_stick(self.position_matrix[0][x], self.position_matrix[-1][x], thickness = thickness).attach_to(base)

class Node(object):
    def __init__(self, grid, height = 0.05, origin_offset = 0.001):
        self.grid = grid
        self.origin_pos_dict = {}
        self.node_infos = {}
        self.node_matrix_infos = {}
        self.origin_offset = origin_offset
        self.height = height
        for y in range(0, self.grid.get_wid()-1):
            for x in range(0, self.grid.get_len()-1):
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
                normal = - hu.normal_from_3point(self.grid.get_origin(x - 1, y), self.grid.get_origin(x + 1, y), self.grid.get_origin(x - 1, y - 1))
                id_infos["height"] = self.grid.get_origin(x, y) + normal * self.height
                id_infos["low"] = self.grid.get_origin(x, y) - normal * self.height

                self.node_infos[f"{x}-{y}"]=id_infos

        self.generate_matrix()
    def generate_matrix(self):
        for y in range(2, self.grid.get_wid() - 2):
            for x in range(2, self.grid.get_len() - 2):
                id = f"{x}-{y}"
                matrix_id_infos = {}
                matrix_id_infos["parity"] = self.node_infos[id]["parity"]
                matrix_id_infos["origin"] = self.node_infos[id]["origin"]
                if self.node_infos[id]["parity"] == "even-even":
                    matrix_id_infos["top"] = self.node_infos[id]["rgt"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["rgt"]) * self.origin_offset
                    matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["lft"]) * self.origin_offset
                    matrix_id_infos["center1"] = self.node_infos[id]["up"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["up"]) * self.origin_offset
                    matrix_id_infos["center2"] = self.node_infos[id]["height"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["height"]) * self.origin_offset
                    matrix_id_infos["center3"] = self.node_infos[id]["down"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["down"]) * self.origin_offset
                    matrix_id_infos["center4"] = self.node_infos[id]["low"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["low"]) * self.origin_offset

                elif self.node_infos[id]["parity"] == "odd-even":
                    matrix_id_infos["top"] = self.node_infos[id]["rgt"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["rgt"]) * self.origin_offset
                    matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["lft"]) * self.origin_offset
                    xy_list = id.split("-")
                    x = int(xy_list[0])
                    y = int(xy_list[1])


                    matrix_id_infos["center1"] = self.node_infos[id]["origin"] + cpt_vec(self.node_infos[f"{x}-{y+1}"]["height"], self.node_infos[f"{x}-{y}"]["origin"], self.origin_offset)
                    matrix_id_infos["center2"] = self.node_infos[id]["origin"] + cpt_vec(self.node_infos[f"{x}-{y -1}"]["height"],self.node_infos[f"{x}-{y}"]["origin"], self.origin_offset)
                    matrix_id_infos["center3"] = self.node_infos[id]["origin"] + cpt_vec(self.node_infos[f"{x}-{y-1}"]["low"], self.node_infos[f"{x}-{y}"]["origin"], self.origin_offset)
                    matrix_id_infos["center4"] = self.node_infos[id]["origin"] + cpt_vec(self.node_infos[f"{x}-{y+1}"]["low"], self.node_infos[f"{x}-{y}"]["origin"], self.origin_offset)

                elif self.node_infos[id]["parity"] == "even-odd":
                    matrix_id_infos["top"] = self.node_infos[id]["up"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["up"]) * self.origin_offset
                    matrix_id_infos["bottom"] = self.node_infos[id]["down"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["down"]) * self.origin_offset

                    xy_list = id.split("-")
                    x = int(xy_list[0])
                    y = int(xy_list[1])

                    matrix_id_infos["center1"] = self.node_infos[id]["origin"] + cpt_vec(
                        self.node_infos[f"{x + 1}-{y}"]["height"], self.node_infos[f"{x}-{y}"]["origin"],
                        self.origin_offset)
                    matrix_id_infos["center2"] = self.node_infos[id]["origin"] + cpt_vec(
                        self.node_infos[f"{x-1}-{y}"]["height"], self.node_infos[f"{x}-{y}"]["origin"],
                        self.origin_offset)
                    matrix_id_infos["center3"] = self.node_infos[id]["origin"] + cpt_vec(
                        self.node_infos[f"{x-1}-{y}"]["low"], self.node_infos[f"{x}-{y}"]["origin"],
                        self.origin_offset)
                    matrix_id_infos["center4"] = self.node_infos[id]["origin"] + cpt_vec(
                        self.node_infos[f"{x+1}-{y}"]["low"], self.node_infos[f"{x}-{y}"]["origin"],
                        self.origin_offset)
                else:
                    pass

                self.node_matrix_infos[id] = matrix_id_infos


class Element(object):
    def __init__(self, node, radius = 0.01, manualset = False, id = None, cut = None, support = None, filename = "41-41"):
        # self.construct()
        self.radius = radius
        self.id = id
        self.parity = node["parity"]
        self.support = support
        self.filename = filename
        if cut:
            a = cm.gen_sphere(node["origin"], radius = 0.001)
            if a.is_mcdwith(objcm_list=cut):
                self.parity = "space"
        if manualset:
            self.t = node["top"]
            self.b = node["bottom"]
            self.c1 = node["center1"]
            self.c2 = node["center2"]
            self.c3 = node["center3"]
            self.c4 = node["center4"]
        else:
            if self.parity == "space":
                pass
            else:
                self.t = node["top"]
                self.b = node["bottom"]
                self.c1 = node["center1"]
                self.c2 = node["center2"]
                self.c3 = node["center3"]
                self.c4 = node["center4"]
                self.construct()
                # self.get_stl()

    def construct(self):
        # self.bar_list = []
        # self.bar_list.append()
        self.t_c1 = capsule_link_start_end(self.t, self.c1, self.radius)
        self.t_c2 = capsule_link_start_end(self.t, self.c2, self.radius)
        self.t_c3 = capsule_link_start_end(self.t, self.c3, self.radius)
        self.t_c4 = capsule_link_start_end(self.t, self.c4, self.radius)
        self.b_c1 = capsule_link_start_end(self.b, self.c1, self.radius)
        self.b_c2 = capsule_link_start_end(self.b, self.c2, self.radius)
        self.b_c3 = capsule_link_start_end(self.b, self.c3, self.radius)
        self.b_c4 = capsule_link_start_end(self.b, self.c4, self.radius)
        self.c1_c2 = capsule_link_start_end(self.c1, self.c2, self.radius)
        self.c2_c3 = capsule_link_start_end(self.c2, self.c3, self.radius)
        self.c3_c4 = capsule_link_start_end(self.c3, self.c4, self.radius)
        self.c4_c1 = capsule_link_start_end(self.c4, self.c1, self.radius)

        self.t_c1.attach_to(base)
        self.t_c2.attach_to(base)
        self.t_c3.attach_to(base)
        self.t_c4.attach_to(base)
        self.b_c1.attach_to(base)
        self.b_c2.attach_to(base)
        self.b_c3.attach_to(base)
        self.b_c4.attach_to(base)
        self.c1_c2.attach_to(base)
        self.c2_c3.attach_to(base)
        self.c3_c4.attach_to(base)
        self.c4_c1.attach_to(base)
        if self.support:
            if self.parity == "even-even":
                self.stage = cylinder_link_start_end(self.c4+np.array([0,0, 0.001]), self.c4+np.array([0,0, -0.0006]), 2.1*self.radius)
                self.stage.attach_to(base)

            elif self.parity == "even-odd":
                # self.pillar_a_1 = cylinder_link_start_end(self.c3 + np.array([0, 0, 0.001]), self.c3 + np.array([0, 0, -(0.005-0.006*0.707)-0.0006]),
                #                                      self.radius+0.0001)
                # self.pillar_a_2 = cylinder_link_start_end(self.c4 + np.array([0, 0, 0.001]),
                #                                           self.c4 + np.array([0, 0, -(0.005-0.006*0.707)-0.0006]),
                #                                            self.radius+0.0001)
                self.pillar_a_1 = cylinder_link_start_end(self.c3,
                                                          np.array([self.c3[0],self.c3[1],-0.0056]),
                                                          self.radius *2.1)
                self.pillar_a_2 = cylinder_link_start_end(self.c4 ,
                                                          np.array([self.c4[0],self.c4[1],-0.0056]),
                                                          self.radius *2.1)
                self.pillar_a_1.attach_to(base)
                self.pillar_a_2.attach_to(base)

            elif self.parity == "odd-even":
                # self.pillar_b_1 = cylinder_link_start_end(self.c3 + np.array([0, 0, 0.001]), self.c3 + np.array([0, 0, -(0.005-0.006*0.707)-0.0006]),
                #                                      self.radius)
                # self.pillar_b_2 = cylinder_link_start_end(self.c4 + np.array([0, 0, 0.001]),
                #                                         self.c4 + np.array([0, 0, -(0.005-0.006*0.707)-0.0006]),
                #                                         self.radius)
                self.pillar_b_1 = cylinder_link_start_end(self.c3,
                                                          np.array([self.c3[0],self.c3[1],-0.0056]),
                                                          self.radius *2.1)
                self.pillar_b_2 = cylinder_link_start_end(self.c4,
                                                          np.array([self.c4[0],self.c4[1],-0.0056]),
                                                          self.radius *2.1)
                self.pillar_b_1.attach_to(base)
                self.pillar_b_2.attach_to(base)
        else:
            pass

    def get_stl(self):
        t_c1_objtrm = self.t_c1.objtrm
        t_c2_objtrm = self.t_c2.objtrm
        t_c3_objtrm = self.t_c3.objtrm
        t_c4_objtrm = self.t_c4.objtrm
        b_c1_objtrm = self.b_c1.objtrm
        b_c2_objtrm = self.b_c2.objtrm
        b_c3_objtrm = self.b_c3.objtrm
        b_c4_objtrm = self.b_c4.objtrm
        c1_c2_objtrm = self.c1_c2.objtrm
        c2_c3_objtrm = self.c2_c3.objtrm
        c3_c4_objtrm = self.c3_c4.objtrm
        c4_c1_objtrm = self.c4_c1.objtrm

        this_dir, this_filename = os.path.split(__file__)
        # file = f"{this_dir}/space_boolean/{self.id}-"
        file = f"{this_dir}/{self.filename}/{self.id}-"

        t_c1_objtrm.export(f"{file}t_c1.stl")
        t_c2_objtrm.export(f"{file}t_c2.stl")
        t_c3_objtrm.export(f"{file}t_c3.stl")
        t_c4_objtrm.export(f"{file}t_c4.stl")
        b_c1_objtrm.export(f"{file}b_c1.stl")
        b_c2_objtrm.export(f"{file}b_c2.stl")
        b_c3_objtrm.export(f"{file}b_c3.stl")
        b_c4_objtrm.export(f"{file}b_c4.stl")
        c1_c2_objtrm.export(f"{file}c1_c2.stl")
        c2_c3_objtrm.export(f"{file}c2_c3.stl")
        c3_c4_objtrm.export(f"{file}c3_c4.stl")
        c4_c1_objtrm.export(f"{file}c4_c1.stl")

        # t_c1_objtrm_temp = trimesh.load("space_boolean/t_c1.stl")
        #
        # t_c2_objtrm_temp = trimesh.load("space_boolean/t_c2.stl")
        # # temp = tb.union([t_c1_objtrm, t_c2_objtrm],engine="blender")
        # t_c3_objtrm_temp = trimesh.load("space_boolean/t_c3.stl")
        # # temp = tb.union([temp, t_c3_objtrm], engine="blender")
        # t_c4_objtrm_temp = trimesh.load("space_boolean/t_c4.stl")
        # # temp = tb.union([temp, t_c4_objtrm], engine="blender")
        # b_c1_objtrm_temp = trimesh.load("space_boolean/b_c1.stl")
        # # temp = tb.union([temp, b_c1_objtrm], engine="blender")
        # b_c2_objtrm_temp = trimesh.load("space_boolean/b_c2.stl")
        # # temp = tb.union([temp, b_c2_objtrm ], engine="blender")
        # b_c3_objtrm_temp = trimesh.load("space_boolean/b_c3.stl")
        # # temp = tb.union([temp, b_c3_objtrm], engine="blender")
        # b_c4_objtrm_temp = trimesh.load("space_boolean/b_c4.stl")
        # # temp = tb.union([temp, b_c4_objtrm], engine="blender")
        # c1_c2_objtrm_temp = trimesh.load("space_boolean/c1_c2.stl")
        # # temp = tb.union([temp, c1_c2_objtrm], engine="blender")
        # c2_c3_objtrm_temp = trimesh.load("space_boolean/c2_c3.stl")
        # # temp = tb.union([temp, c2_c3_objtrm], engine="blender")
        # c3_c4_objtrm_temp = trimesh.load("space_boolean/c3_c4.stl")
        # # temp = tb.union([temp, c3_c4_objtrm], engine="blender")
        # c4_c1_objtrm_temp = trimesh.load("space_boolean/c4_c1.stl")
        # # temp = tb.union([temp, c4_c1_objtrm], engine="blender")

        if self.support:
            if self.parity == "even-even":
                stage_objtrm = self.stage.objtrm
                stage_objtrm.export(f"{file}stage.stl")
            elif self.parity == "even-odd":
                pillar_eo_1 = self.pillar_a_1.objtrm
                pillar_eo_2 = self.pillar_a_2.objtrm
                pillar_eo_1.export(f"{file}pillar_eo_1.stl")
                pillar_eo_2.export(f"{file}pillar_eo_2.stl")
            elif self.parity == "odd-even":
                pillar_oe_1 = self.pillar_b_1.objtrm
                pillar_oe_2 = self.pillar_b_2.objtrm
                pillar_oe_1.export(f"{file}pillar_oe_1.stl")
                pillar_oe_2.export(f"{file}pillar_oe_2.stl")
        else:
            pass
        # temp = tb.union([t_c1_objtrm_temp, t_c2_objtrm_temp, t_c3_objtrm_temp, t_c4_objtrm_temp, b_c1_objtrm_temp, b_c2_objtrm_temp, b_c3_objtrm_temp, b_c4_objtrm_temp],
        #                              engine="blender")
        # temp.export(f"{file}element.stl")
        # cm.CollisionModel("space_boolean/element.stl").attach_to(base)

if __name__ == '__main__':
    base = wd.World(cam_pos=[0.075, 0.08, 0.050], w=960, h=540, lookat_pos=[0, 0, 0.0])
    # gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)

    interval = 0.006
    # len_num = 45+6
    # wid_num = 45+6
    # len_num = 11+4
    # wid_num = 11+4
    len_num =9
    wid_num =9
    matrix = [[np.array([interval*x, interval*y, 0.000]) for x in range(len_num)] for y in range(wid_num)]

    # c1 = cm.gen_box(extent=[.006*18.5, 0.006*9, .001], homomat=rm.homomat_from_posrot([0.006*18,0.006*6.5,0], rm.rotmat_from_axangle([0,0,1], 0*np.pi/2)),rgba=[0,0,0,0.2])
    # c2 = cm.gen_box(extent=[.006*18.5, 0.006*12, .001], homomat=rm.homomat_from_posrot([0.006*18,0.006*29,0], rm.rotmat_from_axangle([0,0,1], 0*np.pi/2)),rgba=[0, 0, 0, 0.2])
    # c3 = cm.gen_box(extent=[.010, .050, .001], homomat=rm.homomat_from_posrot([0, 0.02, 0], rm.rotmat_from_axangle([0, 0, 1], np.pi / 2)),
    #            rgba=[0, 0, 0, 0.2])
    # c4 = cm.gen_box(extent=[.010, .050, .001], homomat=rm.homomat_from_posrot([0.02,0,0], rm.rotmat_from_axangle([0,0,1], 0)), rgba=[0, 0, 0, 0.2])
    #
    # cut_list = [c1,c2]
    # # cut_list = [c1]

    # gm.gen_frame(pos = matrix[0][0], length=.01, thickness=.0005, ).attach_to(base)

    cut_list = []
    for model in cut_list:
        model.attach_to(base)
    grid = Grid(np.array(matrix), interval)
    node = Node(grid, height=0.006, origin_offset=0.001)
    matrix_infos = node.node_matrix_infos

    base.run()
    for id, key in enumerate(matrix_infos.keys()):
        # print("hi", id, key)
        element = Element(matrix_infos[key], radius=0.00045, id=key, cut=cut_list, support=False, filename="11-11-0.9")
        # if id ==1:
        #     element = Element(matrix_infos[key], radius=0.00045, id = key, cut = cut_list, support = False, filename ="11-11-0.9")
        #     gm.gen_sphere(pos=element.t, radius=0.0006, rgba=(0, 1, 1, 1)).attach_to(base)
        #     gm.gen_sphere(pos=element.b, radius=0.0006, rgba=(0, 1, 1, 1)).attach_to(base)
        #     gm.gen_sphere(pos=element.c1, radius=0.0006, rgba=(0, 1, 1, 1)).attach_to(base)
        #     gm.gen_sphere(pos=element.c2, radius=0.0006, rgba=(0, 1, 1, 1)).attach_to(base)
        #     gm.gen_sphere(pos=element.c3, radius=0.0006, rgba=(0, 1, 1, 1)).attach_to(base)
        #     gm.gen_sphere(pos=element.c4, radius=0.0006, rgba=(0, 1, 1, 1)).attach_to(base)
        # element.get_stl()
        # break
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