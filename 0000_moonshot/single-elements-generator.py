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
import trimesh.boolean as tb
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_sim.robots.ur3e_dual.ur3e_dual as ur3ed
import robot_sim.robots.sda5f.sda5f as sda5
import motion.probabilistic.rrt_connect as rrtc
import robot_sim._kinematics.collision_checker as cc
import robot_sim.end_effectors.grippers.gripper_interface as gi
# import robot_sim.
import os
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
        print(self.len_num, self.wid_num)
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
    def __init__(self, grid, height = 0.05, origin_offset = 0.001):
        self.grid = grid
        self.origin_pos_dict = {}
        self.node_infos = {}
        self.node_matrix_infos = {}
        self.origin_offset = origin_offset
        self.height = height
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
                matrix_id_infos["center2"] = self.node_infos[id]["origin"] + np.array([0, 0, self.height-self.origin_offset])
                matrix_id_infos["center3"] = self.node_infos[id]["down"] + np.array([0, self.origin_offset, 0])
                matrix_id_infos["center4"] = self.node_infos[id]["origin"] + np.array([0, 0, -self.height+self.origin_offset])
            elif self.node_infos[id]["parity"] == "odd-even":
                matrix_id_infos["top"] = self.node_infos[id]["rgt"] + np.array([-self.origin_offset, 0, 0])
                matrix_id_infos["bottom"] = self.node_infos[id]["lft"] + np.array([self.origin_offset, 0, 0])
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
                matrix_id_infos["top"] = self.node_infos[id]["up"] + np.array([0, -self.origin_offset, 0])
                matrix_id_infos["bottom"] = self.node_infos[id]["down"] + np.array([0, self.origin_offset, 0])
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
    def __init__(self, node, radius = 0.01, manualset = False, id = None):
        # self.construct()
        self.radius = radius
        self.id = id
        if manualset:
            self.t = node["top"]
            self.b = node["bottom"]
            self.c1 = node["center1"]
            self.c2 = node["center2"]
            self.c3 = node["center3"]
            self.c4 = node["center4"]
        else:
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
                self.get_stl()

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
        file = f"{this_dir}/space_boolean/{self.id}-"

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

        t_c1_objtrm_temp = trimesh.load("space_boolean/t_c1.stl")

        t_c2_objtrm_temp = trimesh.load("space_boolean/t_c2.stl")
        # temp = tb.union([t_c1_objtrm, t_c2_objtrm],engine="blender")
        t_c3_objtrm_temp = trimesh.load("space_boolean/t_c3.stl")
        # temp = tb.union([temp, t_c3_objtrm], engine="blender")
        t_c4_objtrm_temp = trimesh.load("space_boolean/t_c4.stl")
        # temp = tb.union([temp, t_c4_objtrm], engine="blender")
        b_c1_objtrm_temp = trimesh.load("space_boolean/b_c1.stl")
        # temp = tb.union([temp, b_c1_objtrm], engine="blender")
        b_c2_objtrm_temp = trimesh.load("space_boolean/b_c2.stl")
        # temp = tb.union([temp, b_c2_objtrm ], engine="blender")
        b_c3_objtrm_temp = trimesh.load("space_boolean/b_c3.stl")
        # temp = tb.union([temp, b_c3_objtrm], engine="blender")
        b_c4_objtrm_temp = trimesh.load("space_boolean/b_c4.stl")
        # temp = tb.union([temp, b_c4_objtrm], engine="blender")
        c1_c2_objtrm_temp = trimesh.load("space_boolean/c1_c2.stl")
        # temp = tb.union([temp, c1_c2_objtrm], engine="blender")
        c2_c3_objtrm_temp = trimesh.load("space_boolean/c2_c3.stl")
        # temp = tb.union([temp, c2_c3_objtrm], engine="blender")
        c3_c4_objtrm_temp = trimesh.load("space_boolean/c3_c4.stl")
        # temp = tb.union([temp, c3_c4_objtrm], engine="blender")
        c4_c1_objtrm_temp = trimesh.load("space_boolean/c4_c1.stl")
        # temp = tb.union([temp, c4_c1_objtrm], engine="blender")

        # temp = tb.union([t_c1_objtrm_temp, t_c2_objtrm_temp, t_c3_objtrm_temp, t_c4_objtrm_temp, b_c1_objtrm_temp, b_c2_objtrm_temp, b_c3_objtrm_temp, b_c4_objtrm_temp],
        #                              engine="blender")
        # temp.export(f"{file}element.stl")
        # cm.CollisionModel("space_boolean/element.stl").attach_to(base)


if __name__ == '__main__':
    base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960,
                    h=540, lookat_pos=[0, 0, 0.0])
    gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)

    interval = 0.007
    len_num = 3
    wid_num = 3
    matrix = [[np.array([interval*x, interval*y, 0.000]) for x in range(len_num)] for y in range(wid_num)]

    grid = Grid(np.array(matrix), interval)
    node = Node(grid, height=0.007, origin_offset=0.0012)
    matrix_infos = node.node_matrix_infos
    for key in matrix_infos.keys():
        element = Element(matrix_infos[key], radius=0.0007, id = key)
        # element.get_stl()

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