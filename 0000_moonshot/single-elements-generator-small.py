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
    capsule = cm.gen_capsule(spos=start, epos=end, radius=radius, section=[5, 5],  rgba = rgba)
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
        for y in range(self.wid_num):
            for x in range(self.len_num):
                gm.gen_sphere(pos = self.position_matrix[y][x], radius = 0.0003, rgba=(0,0,0,1)).attach_to(base)
                if x == self.len_num - 1 and y < self.wid_num - 1:
                    # print("check")
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y + 1],
                                 thickness=thickness).attach_to(base)
                elif y == self.wid_num - 1 and x < self.len_num-1:
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x + 1][y],
                                 thickness=thickness).attach_to(base)
                elif x < self.len_num-1 and y < self.wid_num - 1:
                    print("x:",x,"====", "y:",y)
                    print(self.position_matrix.shape)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x+1][y], thickness=thickness).attach_to(
                        base)
                    gm.gen_stick(self.position_matrix[x][y], self.position_matrix[x][y+1], thickness=thickness).attach_to(
                        base)
                # elif x == self.len_num - 1 and y < self.wid_num - 1:
                #     pass

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

                    matrix_id_infos["origin_sen"] = matrix_id_infos["origin"]+np.array([0,0,self.height])

                    def rot_point(point, center, angle):
                        frame_origin_sen = np.linalg.inv(rm.homomat_from_posrot(center))
                        point = rm.homomat_transform_points(
                            rm.homomat_from_posrot([0, 0, 0], rm.rotmat_from_axangle([0, 0, 1], angle)),
                            rm.homomat_transform_points(frame_origin_sen, point))
                        point = rm.homomat_transform_points(
                            rm.homomat_from_posrot(matrix_id_infos["origin_sen"]), point)
                        return point
                    matrix_id_infos["top_sen"] = self.node_infos[id]["rgt"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["rgt"]) * self.origin_offset+np.array([0,0,self.height])
                    matrix_id_infos["bottom_sen"] = self.node_infos[id]["lft"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["lft"]) * self.origin_offset+np.array([0,0,self.height])
                    matrix_id_infos["center1_sen"] = self.node_infos[id]["up"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["up"]) * self.origin_offset+np.array([0,0,self.height])
                    matrix_id_infos["center2_sen"] = self.node_infos[id]["height"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["height"]) * self.origin_offset+np.array([0,0,self.height])
                    matrix_id_infos["center3_sen"] = self.node_infos[id]["down"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["down"]) * self.origin_offset+np.array([0,0,self.height])
                    matrix_id_infos["center4_sen"] = self.node_infos[id]["low"] + rm.unit_vector(
                        self.node_infos[id]["origin"] - self.node_infos[id]["low"]) * self.origin_offset+np.array([0,0,self.height])
                    matrix_id_infos["bottom_sen"] = rot_point(matrix_id_infos["bottom_sen"], matrix_id_infos["origin_sen"], np.pi*0.25)
                    matrix_id_infos["top_sen"] = rot_point(matrix_id_infos["top_sen"],
                                                              matrix_id_infos["origin_sen"], np.pi * 0.25)
                    matrix_id_infos["center1_sen"] = rot_point(matrix_id_infos["center1_sen"],
                                                              matrix_id_infos["origin_sen"], np.pi * 0.25)
                    matrix_id_infos["center3_sen"] = rot_point(matrix_id_infos["center3_sen"],
                                                               matrix_id_infos["origin_sen"], np.pi * 0.25)
                    # matrix_id_infos["bottom_sen"] = rot_point(matrix_id_infos["bottom_sen"],
                    #                                           matrix_id_infos["origin_sen"], np.pi * 0.25)
                    # gm.gen_sphere(matrix_id_infos["top_sen"],radius=0.002).attach_to(base)
                    # gm.gen_sphere(matrix_id_infos["bottom_sen"], radius=0.002).attach_to(base)


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
    def __init__(self, node, dict = "circ", radius = 0.01, manualset = False, id = None, cut = None, support = None,secondlayer = False):
        self.radius = radius
        self.id = id
        self.node = node
        self.parity = node["parity"]
        self.support = support
        self.secondlayer = secondlayer
        if cut:
            a = cm.gen_sphere(node["origin"], radius = 0.001)
            if a.is_mcdwith(objcm_list=cut):
                self.parity = "space"
        if manualset:
            self.o = node["origin"]
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
                self.o = node["origin"]
                self.t = node["top"]
                self.b = node["bottom"]
                self.c1 = node["center1"]
                self.c2 = node["center2"]
                self.c3 = node["center3"]
                self.c4 = node["center4"]
                if secondlayer:
                    if "origin_sen" in node.keys():
                        self.o_sen = node["origin_sen"]
                        self.t_sen = node["top_sen"]
                        self.b_sen = node["bottom_sen"]
                        self.c1_sen = node["center1_sen"]
                        self.c2_sen = node["center2_sen"]
                        self.c3_sen = node["center3_sen"]
                        self.c4_sen = node["center4_sen"]
                self.construct()
                self.get_stl(dict)

    def construct(self):
        if self.parity == "even-even":
            self.t1 = gm.gen_ellipse(self.o,[self.t, self.c1,self.b,self.c3], self.radius, 12)
            self.t2 = gm.gen_ellipse(self.o, [self.c2, self.c1, self.c4, self.c3], self.radius, 12)
            self.t3 = gm.gen_ellipse(self.o, [self.t, self.c2, self.b, self.c4], self.radius, 12)
            # print("check", [self.c1, self.c2,self.c3,self.c4])
            self.t1.attach_to(base)
            self.t2.attach_to(base)
            # print("check", [self.c1, self.c2, self.c3, self.c4])
            self.t3.attach_to(base)


        else:
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

        if self.secondlayer and "origin_sen" in self.node.keys():
            self.t_c1_sen = capsule_link_start_end(self.t_sen, self.c1_sen, self.radius)
            self.t_c2_sen = capsule_link_start_end(self.t_sen, self.c2_sen, self.radius)
            self.t_c3_sen = capsule_link_start_end(self.t_sen, self.c3_sen, self.radius)
            self.t_c4_sen = capsule_link_start_end(self.t_sen, self.c4_sen, self.radius)
            self.b_c1_sen = capsule_link_start_end(self.b_sen, self.c1_sen, self.radius)
            self.b_c2_sen = capsule_link_start_end(self.b_sen, self.c2_sen, self.radius)
            self.b_c3_sen = capsule_link_start_end(self.b_sen, self.c3_sen, self.radius)
            self.b_c4_sen = capsule_link_start_end(self.b_sen, self.c4_sen, self.radius)
            self.c1_c2_sen = capsule_link_start_end(self.c1_sen, self.c2_sen, self.radius)
            self.c2_c3_sen = capsule_link_start_end(self.c2_sen, self.c3_sen, self.radius)
            self.c3_c4_sen = capsule_link_start_end(self.c3_sen, self.c4_sen, self.radius)
            self.c4_c1_sen = capsule_link_start_end(self.c4_sen, self.c1_sen, self.radius)
            self.t_c1_sen.attach_to(base)
            self.t_c2_sen.attach_to(base)
            self.t_c3_sen.attach_to(base)
            self.t_c4_sen.attach_to(base)
            self.b_c1_sen.attach_to(base)
            self.b_c2_sen.attach_to(base)
            self.b_c3_sen.attach_to(base)
            self.b_c4_sen.attach_to(base)
            self.c1_c2_sen.attach_to(base)
            self.c2_c3_sen.attach_to(base)
            self.c3_c4_sen.attach_to(base)
            self.c4_c1_sen.attach_to(base)




        self.supportoffset = 0.015
        if self.support:
            if self.parity == "even-even":

                if self.id.endswith('2'):
                    self.stage1 = cylinder_link_start_end(self.c3 + np.array([0, -0.001, 0]),
                                                         np.array([self.c3[0], self.c3[1] + 0.001, self.c3[2]]),
                                                         2.1 * self.radius)
                    self.stage2 = cylinder_link_start_end(self.c3 + np.array([0, 0, 0]),
                                                         np.array([self.c3[0], self.c3[1] + 0.006, self.c3[2]]),
                                                         1.2 * self.radius)
                    self.stage1.attach_to(base)

                else:
                    self.stage1 = cylinder_link_start_end(self.c3 + np.array([0, -0.002, 0]),
                                                          np.array([self.c3[0], self.c3[1] + 0.001, self.c3[2]]),
                                                          1.2 * self.radius)
                    self.stage2 = cylinder_link_start_end(self.c3 + np.array([0, -0.002, 0]),
                                                          np.array([self.c3[0], self.c3[1] + 0.001, self.c3[2]]),
                                                          1.2 * self.radius)
                    self.stage1.attach_to(base)

            elif self.parity == "even-odd":
                if self.id.endswith("7"):
                    self.pillar_a_1 = cylinder_link_start_end(self.c3 + np.array([0, -0.001, 0]),
                                                          np.array([self.c3[0], self.c3[1] + 0.001, self.c3[2]]),
                                                          2.1 * self.radius)
                    self.pillar_a_2 = cylinder_link_start_end(self.c3 + np.array([0, 0, 0]),
                                                          np.array([self.c3[0], self.c3[1] + 0.006, self.c3[2]]),
                                                          1.2 * self.radius)
                else:
                    self.pillar_a_1 = cylinder_link_start_end(self.t + np.array([0, -0.001, 0]),
                                                              self.t + np.array([0, 0.002, 0]),
                                                              1.2 * self.radius)
                    self.pillar_a_2 = cylinder_link_start_end(self.t + np.array([0, -0.001, 0]),
                                                              self.t + np.array([0, 0.002, 0]),
                                                              1.2 * self.radius)
                    self.pillar_a_1.attach_to(base)
                    self.pillar_a_2.attach_to(base)

            elif self.parity == "odd-even":
                self.pillar_b_1 = cylinder_link_start_end(self.c2 + np.array([0,0.000, 0]), self.c2 + np.array([0,-0.0045, 0]),
                                                     self.radius)
                self.pillar_b_2 = cylinder_link_start_end(self.c3 + np.array([0,0.000, 0]), self.c3 + np.array([0,-0.0045, 0]),
                                                     self.radius)
                if self.id.endswith('-2'):
                    self.pillar_b_1 = cylinder_link_start_end(np.array([self.c2[0], self.c2[1], self.c2[2]]),
                                                              np.array([self.c2[0], 0.5*(self.c2[1]+self.c4[1])-0.006, self.c2[2]]),
                                                              2.1*self.radius)
                    self.pillar_b_2 = cylinder_link_start_end(np.array([self.c3[0], self.c3[1], self.c3[2]]),
                                                              np.array([self.c3[0], 0.5*(self.c3[1]+self.c4[1])-0.006, self.c3[2]]),
                                                              2.1*self.radius)

                self.pillar_b_1.attach_to(base)
                self.pillar_b_2.attach_to(base)
        else:
            pass

    def get_stl(self, dict):

        this_dir, this_filename = os.path.split(__file__)
        # file = f"{this_dir}/space_boolean/{self.id}-"
        file = f"{this_dir}/{dict}/{self.id}-"

        if self.parity == "even-even":
            t1_objtrm = self.t1.objtrm
            t2_objtrm = self.t2.objtrm
            t3_objtrm = self.t3.objtrm
            t1_objtrm.export(f"{file}t_c1.stl")
            t2_objtrm.export(f"{file}t_c2.stl")
            t3_objtrm.export(f"{file}t_c3.stl")

        else:
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

        if self.secondlayer and "origin_sen" in self.node.keys():
            t_c1_sen_objtrm = self.t_c1_sen.objtrm
            t_c2_sen_objtrm = self.t_c2_sen.objtrm
            t_c3_sen_objtrm = self.t_c3_sen.objtrm
            t_c4_sen_objtrm = self.t_c4_sen.objtrm
            b_c1_sen_objtrm = self.b_c1_sen.objtrm
            b_c2_sen_objtrm = self.b_c2_sen.objtrm
            b_c3_sen_objtrm = self.b_c3_sen.objtrm
            b_c4_sen_objtrm = self.b_c4_sen.objtrm
            c1_c2_sen_objtrm = self.c1_c2_sen.objtrm
            c2_c3_sen_objtrm = self.c2_c3_sen.objtrm
            c3_c4_sen_objtrm = self.c3_c4_sen.objtrm
            c4_c1_sen_objtrm = self.c4_c1_sen.objtrm
            t_c1_sen_objtrm.export(f"{file}t_c1_sen.stl")
            t_c2_sen_objtrm.export(f"{file}t_c2_sen.stl")
            t_c3_sen_objtrm.export(f"{file}t_c3_sen.stl")
            t_c4_sen_objtrm.export(f"{file}t_c4_sen.stl")
            b_c1_sen_objtrm.export(f"{file}b_c1_sen.stl")
            b_c2_sen_objtrm.export(f"{file}b_c2_sen.stl")
            b_c3_sen_objtrm.export(f"{file}b_c3_sen.stl")
            b_c4_sen_objtrm.export(f"{file}b_c4_sen.stl")
            c1_c2_sen_objtrm.export(f"{file}c1_c2_sen.stl")
            c2_c3_sen_objtrm.export(f"{file}c2_c3_sen.stl")
            c3_c4_sen_objtrm.export(f"{file}c3_c4_sen.stl")
            c4_c1_sen_objtrm.export(f"{file}c4_c1_sen.stl")

        if self.support:
            if self.parity == "even-even":
                stage1_objtrm = self.stage1.objtrm
                stage2_objtrm = self.stage2.objtrm
                stage1_objtrm.export(f"{file}stage1.stl")
                stage2_objtrm.export(f"{file}stage2.stl")
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


if __name__ == '__main__':
    base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960, h=540, lookat_pos=[0, 0, 0.0])
    gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)



    # p=np.array([[0.010, 0,0],[0.007,0.007,0],[0,0.01,0],[-0.007,0.007,0],[-0.01,0,0],[-0.007,-0.007,0],[0,-0.01,0],[0.007,-0.007,0],[0.010, 0 ,0],[0.007,0.007,0]])
    # p = np.array(
    #     [[0.010, 0, 0], [0.007, 0.007, 0], [0, 0.01, 0], [-0.007, 0.007, 0], [-0.01, 0, 0], [-0.007, -0.007, 0],
    #      [0, -0.01, 0]])
    # a = gm.gen_curveline(p,0.001, 16)
    # a.set_rgba([1,0,0,1])
    # a.attach_to(base)
    # center = np.array([0.0+0.05,0.0+0.05,0.0+0.05])
    # points = np.array([[0.02+0.05,0+0.05,0+0.05],[0+0.05,0.01+0.05,0.01+0.05],[-0.02+0.05,0+0.05,0+0.05],[0+0.05,-0.01+0.05,0.01+0.05]])
    # gm.gen_ellipse(center,points,0.001,15).attach_to(base)

    interval = 0.005
    len_num = 27
    wid_num = 27
    # len_num =9
    # wid_num = 9
    n = 32
    r = 0.04
    height_layer = 32
    matrix = [[np.array([interval * x, interval * y, 0.000]) for x in range(len_num)] for y in range(wid_num)]
    matrix_second = [[np.array([interval * x, interval * y, 2*interval]) for x in range(len_num)] for y in range(wid_num)]
    cut_list = []

    grid = Grid(np.array(matrix), interval)
    node = Node(grid, height=0.003, origin_offset=0.0008)
    matrix_infos = node.node_matrix_infos
    for key in matrix_infos.keys():
        element = Element(matrix_infos[key], dict="3-3",radius=0.0006, id = key, cut = cut_list, support = False, secondlayer = False)

    # grid_double = Grid(np.array(matrix_second), interval)
    # node_double = Node(grid_double , height=0.006, origin_offset=0.001)
    # matrix_infos = node_double.node_matrix_infos
    # for key in matrix_infos.keys():
    #     element = Element(matrix_infos[key], dict="5-5", radius=0.0006, id=key, cut=cut_list, support=False,
    #                       secondlayer=False)

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