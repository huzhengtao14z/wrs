import math

import basis.trimesh as trimesh
from panda3d.core import NodePath
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
import basis.data_adapter as da
# import utiltools.misc.p3dutils as p3du
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

def drawanySingleSurface(a,vertices,color):
    '''
    draw a surface using a calculated fourth point to creat a hull
    :param base:
    :param vertices:
    :param faces:
    :param color:
    :return:
    '''
    # print("faces in plotsurface",faces)

    surface_vertices = vertices
    # surface = humath.centerPoftrangle(surface_vertices[0][:3], surface_vertices[1][:3], surface_vertices[2][:3])
    surface = trimesh.Trimesh(surface_vertices)
    surface = surface.convex_hull
    surface = gm.GeometricModel(surface)
    if color == "red":
        rgba = (1,0,0,1)
    elif color == "green":
        rgba = (61/255,145/255,64/255,1)
    elif color == "orange":
        rgba = (255/255,100/255,24/255,1)
    elif color == "white":
        rgba = (1,1,1,1)
    elif color == "blue":
        # rgba = (3/255,168/255,158/255,1)
        rgba = (0 / 255, 0/ 255, 255/ 255, 1)
    elif color == "yellow":
        rgba = (1,1,0,1)
    else:
        rgba = (0,0,0,1)
    surface.set_rgba(rgba)
    surface.objpdnp.reparentTo(a)
    # a.reparentTo(base.render)

class Cube(object):
    def __init__(self, node, color = ((1,0,0,1),(1,0,0,1),(1,0,0,1),(1,0,0,1),(1,0,0,1),(1,0,0,1)), pos = [0,0,0]):
        # self.globle_pos = {"a":[[0.01,0.01,-0.010],[0.010,-0.010,-0.010],[-0.010,-0.010,-0.010],[-0.01,0.010,-0.010]],
        #                    "b":[[0.01,0.010, -0.010], [0.01,0.010, 0.010], [-0.01,0.010, 0.010], [-0.01,0.010, -0.010]],
        #                     "c":[[0.01,-0.010, -0.010], [0.01,-0.010, 0.010], [-0.01,-0.010, 0.010], [-0.01,-0.010, -0.010]],
        #                    "d":[[0.01,0.01,0.010],[0.010,-0.010,0.010],[-0.010,-0.010,0.010],[-0.01,0.010,0.010]],
        #                    "e":[[0.01,0.010, -0.010], [0.01,0.010, 0.010], [0.01,-0.010, -0.010], [0.01,-0.010, 0.010]],
        #                    "f":[[-0.01,0.010, -0.010], [-0.01,0.010, 0.010], [-0.01,-0.010, -0.010], [-0.01,-0.010, 0.010]]}
        # xia youqian zuohou shang zuoqian youhou
        self.cubenode = NodePath("cubenode")
        self.node = node
        self.pos = [0,0,0]
        # xia
        self.a = drawanySingleSurface(self.cubenode, [[0.01,0.01,-0.010],[0.010,-0.010,-0.010],[-0.010,-0.010,-0.010],[-0.01,0.010,-0.010]], color=color[0])
        # youqian
        self.b = drawanySingleSurface(self.cubenode, [[0.01,0.010, -0.010], [0.01,0.010, 0.010], [-0.01,0.010, 0.010], [-0.01,0.010, -0.010]], color=color[1])
        # zuohou
        self.c = drawanySingleSurface(self.cubenode, [[0.01,-0.010, -0.010], [0.01,-0.010, 0.010], [-0.01,-0.010, 0.010], [-0.01,-0.010, -0.010]], color=color[2])
        # shang
        self.d = drawanySingleSurface(self.cubenode, [[0.01,0.01,0.010],[0.010,-0.010,0.010],[-0.010,-0.010,0.010],[-0.01,0.010,0.010]], color=color[3])
        # zuoqian
        self.e = drawanySingleSurface(self.cubenode, [[0.01,0.010, -0.010], [0.01,0.010, 0.010], [0.01,-0.010, -0.010], [0.01,-0.010, 0.010]], color=color[4])
        # youhou
        self.f = drawanySingleSurface(self.cubenode, [[-0.01,0.010, -0.010], [-0.01,0.010, 0.010], [-0.01,-0.010, -0.010], [-0.01,-0.010, 0.010]], color=color[5])
        self.cubenode.reparentTo(node)

    def set_node(self, node):
        self.node = node

    def set_pos(self, pos):
        self.pos = pos
        self.cubenode.setPos(self.pos)

    def get_pos(self):
        # return self.pos
        # print("pose", self.pos)
        # print("getpose", da.pdv3_to_npv3(self.cubenode.get_pos(self.node)))

        return da.pdv3_to_npv3(self.cubenode.get_pos(self.node))
    # def set_pos(self, pos):

class Rubik(object):
    def __init__(self):
        self.node_list = []
        self.cube_list = []
        self.b00 = NodePath("normal")
        self.b10 = NodePath("normal")
        self.b20 = NodePath("normal")
        self.b01 = NodePath("normal")
        self.b11 = NodePath("normal")
        self.b21 = NodePath("normal")
        self.b02 = NodePath("normal")
        self.b12 = NodePath("normal")
        self.b22 = NodePath("normal")
        self.c00 = NodePath("normal")
        self.c10 = NodePath("normal")
        self.c20 = NodePath("normal")
        self.c01 = NodePath("normal")
        self.c11 = NodePath("normal")
        self.c21 = NodePath("normal")
        self.c02 = NodePath("normal")
        self.c12 = NodePath("normal")
        self.c22 = NodePath("normal")
        self.t00 = NodePath("normal")
        self.t10 = NodePath("normal")
        self.t20 = NodePath("normal")
        self.t01 = NodePath("normal")
        self.t11 = NodePath("normal")
        self.t21 = NodePath("normal")
        self.t02 = NodePath("normal")
        self.t12 = NodePath("normal")
        self.t22 = NodePath("normal")
        self.node_list = [self.b00,self.b01,self.b02,self.b10,self.b11,self.b12,self.b20,self.b21,self.b22,
                          self.c00,self.c01,self.c02,self.c10,self.c11,self.c12,self.c20,self.c21,self.c22,
                          self.t00,self.t01,self.t02,self.t10,self.t11,self.t12,self.t20,self.t21,self.t22]
        # xia youqian zuohou shang zuoqian youhou

        self.b00_cube = Cube(self.b00, color=("white", "black", "orange", "black", "black", "green"))
        # self.b00_cube.set_pos((-0.02, -0.02, -0.020))
        self.b00.setPos((-0.02, -0.02, -0.020))
        self.cube_list.append(self.b00_cube)

        self.b10_cube = Cube(self.b10, color=("white", "black", "orange", "black", "black", "black"))
        # self.b10_cube.set_pos((0.00, -0.02, -0.020))
        self.b10.setPos((0.00, -0.02, -0.020))
        self.cube_list.append(self.b10_cube)

        self.b20_cube = Cube(self.b20, color=("white", "black", "orange", "black", "blue", "black"))
        # self.b20_cube.set_pos((0.02, -0.02, -0.020))
        self.b20.setPos((0.02, -0.02, -0.020))
        self.cube_list.append(self.b20_cube)


        self.b01_cube = Cube(self.b01, color=("white", "black", "black", "black", "black", "green"))
        # self.b01_cube.set_pos((-0.02, 0.0, -0.020))
        self.b01.setPos((-0.02, 0.0, -0.020))
        self.cube_list.append(self.b01_cube)


        self.b11_cube = Cube(self.b11, color=("white", "black", "black", "black", "black", "black"))
        # self.b11_cube.set_pos((0.0, 0.0, -0.020))
        self.b11.setPos((0.0, 0.0, -0.020))
        self.cube_list.append(self.b11_cube)


        self.b21_cube = Cube(self.b21, color=("white", "black", "orange", "black", "blue", "green"))
        # self.b21_cube.set_pos((0.02, 0.0, -0.020))
        self.b21.setPos((0.02, 0.0, -0.020))
        self.cube_list.append(self.b20_cube)


        self.b02_cube = Cube(self.b02, color=("white", "red", "black", "black", "black", "green"))
        # self.b02_cube.set_pos((-0.02, 0.02, -0.020))
        self.b02.setPos((-0.02, 0.02, -0.020))
        self.cube_list.append(self.b02_cube)


        self.b12_cube = Cube(self.b12, color=("white", "red", "black", "black", "black", "black"))
        # self.b12_cube.set_pos((0.0, 0.02, -0.020))
        self.b12.setPos((0.0, 0.02, -0.020))
        self.cube_list.append(self.b12_cube)


        self.b22_cube = Cube(self.b22, color=("white", "red", "black", "black", "blue", "black"))
        # self.b22_cube.set_pos((0.02, 0.02, -0.020))
        self.b22.setPos((0.02, 0.02, -0.020))
        self.cube_list.append(self.b22_cube)


        self.c00_cube = Cube(self.c00, color=("black", "black", "orange", "black", "black", "green"))
        # self.c00_cube.set_pos((-0.02, -0.02, -0.00))
        self.c00.setPos((-0.02, -0.02, -0.00))
        self.cube_list.append(self.c00_cube)


        self.c10_cube = Cube(self.c10, color=("black", "black", "orange", "black", "black", "black"))
        # self.c10_cube.set_pos((0.00, -0.02, -0.00))
        self.c10.setPos((0.00, -0.02, -0.00))
        self.cube_list.append(self.c10_cube)


        self.c20_cube = Cube(self.c20, color=("black", "black", "orange", "black", "blue", "black"))
        # self.c20_cube.set_pos((0.02, -0.02, -0.00))
        self.c20.setPos((0.02, -0.02, -0.00))
        self.cube_list.append(self.c20_cube)


        self.c01_cube = Cube(self.c01, color=("black", "black", "black", "black", "black", "green"))
        # self.c01_cube.set_pos((-0.02, 0.0, -0.00))
        self.c01.setPos((-0.02, 0.0, -0.00))
        self.cube_list.append(self.c01_cube)


        self.c11_cube = Cube(self.c11, color=("black", "black", "black", "black", "black", "black"))
        # self.c11_cube.set_pos((0.0, 0.0, -0.00))
        self.c11.setPos((0.0, 0.0, -0.00))
        self.cube_list.append(self.c11_cube)


        self.c21_cube = Cube(self.c21, color=("black", "black", "black", "black", "blue", "black"))
        # self.c21_cube.set_pos((0.02, 0.0, -0.00))
        self.c21.setPos((0.02, 0.0, -0.00))
        self.cube_list.append(self.c21_cube)


        self.c02_cube = Cube(self.c02, color=("black", "red", "black", "black", "black", "green"))
        # self.c02_cube.set_pos((-0.02, 0.02, -0.00))
        self.c02.setPos((-0.02, 0.02, -0.00))
        self.cube_list.append(self.c02_cube)


        self.c12_cube = Cube(self.c12, color=("black", "red", "black", "black", "black", "black"))
        # self.c12_cube.set_pos((0.0, 0.02, -0.00))
        self.c12.setPos((0.0, 0.02, -0.00))
        self.cube_list.append(self.c12_cube)


        self.c22_cube = Cube(self.c22, color=("black", "red", "black", "black", "blue", "black"))
        # self.c22_cube.set_pos((0.02, 0.02, -0.00))
        self.c22.setPos((0.02, 0.02, -0.00))
        self.cube_list.append(self.c22_cube)


        self.t00_cube = Cube(self.t00, color=("black", "black", "orange", "yellow", "black", "green"))
        # self.t00_cube.set_pos((-0.02, -0.02, 0.020))
        self.t00.setPos((-0.02, -0.02, 0.020))
        self.cube_list.append(self.t00_cube)


        self.t10_cube = Cube(self.t10, color=("black", "black", "orange", "yellow", "black", "black"))
        # self.t10_cube.set_pos((0.00, -0.02, 0.020))
        self.t10.setPos((0.00, -0.02, 0.020))
        self.cube_list.append(self.t10_cube)


        self.t20_cube = Cube(self.t20, color=("black", "black", "orange", "yellow", "blue", "black"))
        # self.t20_cube.set_pos((0.02, -0.02, 0.020))
        self.t20.setPos((0.02, -0.02, 0.020))
        self.cube_list.append(self.t20_cube)


        self.t01_cube = Cube(self.t01, color=("black", "black", "black", "yellow", "black", "green"))
        # self.t01_cube.set_pos((-0.02, 0.0, 0.020))
        self.t01.setPos((-0.02, 0.0, 0.020))
        self.cube_list.append(self.t01_cube)


        self.t11_cube = Cube(self.t11, color=("black", "black", "black", "yellow", "black", "black"))
        # self.t11_cube.set_pos((0.0, 0.0, 0.020))
        self.t11.setPos((0.0, 0.0, 0.020))
        self.cube_list.append(self.t11_cube)


        self.t21_cube = Cube(self.t21, color=("black", "black", "black", "yellow", "blue", "black"))
        # self.t21_cube.set_pos((0.02, 0.0, 0.020))
        self.t21.setPos((0.02, 0.0, 0.020))
        self.cube_list.append(self.t21_cube)


        self.t02_cube = Cube(self.t02, color=("black", "red", "black", "yellow", "black", "green"))
        # self.t02_cube.set_pos((-0.02, 0.02, 0.020))
        self.t02.setPos((-0.02, 0.02, 0.020))
        self.cube_list.append(self.t02_cube)


        self.t12_cube = Cube(self.t12, color=("black", "red", "black", "yellow", "black", "black"))
        # self.t12_cube.set_pos((0.0, 0.02, 0.020))
        self.t12.setPos((0.0, 0.02, 0.020))
        self.cube_list.append(self.t12_cube)


        self.t22_cube = Cube(self.t22, color=("black", "red", "black", "yellow", "blue", "black"))
        # self.t22_cube.set_pos((0.02, 0.02, 0.020))
        self.t22.setPos((0.02, 0.02, 0.020))
        self.cube_list.append(self.t22_cube)

        # self.show_norot()
        # self.rot_bottom(90)
        # self.rot_rgt(90)
        # self.rot_top(90)
        # self.rot_rgt(90)
        # self.rot_bottom(90)
        # self.rot_lft(45)

        # self.rot_rgt(-45)
        # self.bottom = NodePath("no")
        # # self.bottom.setR((0.0,0.0,-0.02))
        # rot = rm.rotmat_from_axangle((0,0,1),90/180*np.pi)
        # mat = rm.homomat_from_posrot((0,0,0),rot)
        # mat = da.npv3mat3_to_pdmat4((0,0,0), rot)
        # self.bottom.setMat(mat)
        # self.bottom.reparentTo(base.render)
        # self.prepare_rotgroup()
    def attach_to(self, nodepath):
        for cube in self.node_list:
            cube.reparentTo(nodepath)
    def show_norot(self):
        for cube in self.node_list:
            cube.reparentTo(base.render)

    def prepare_rotrightgroup(self, group):
        for i,cube in enumerate(self.cube_list):
            if cube.get_pos()[1]==0.02:
                self.node_list[i].reparentTo(group)

    def rot_bottom(self, angle):
        rot = rm.rotmat_from_axangle((0, 0, 1), angle / 180 * np.pi)
        mat = da.npv3mat3_to_pdmat4((0, 0, 0), rot)
        for i,cube in enumerate(self.node_list):
            if round(cube.getPos()[2], 2)==-0.02:
                print(i)
                self.node_list[i].setMat(da.npmat4_to_pdmat4(np.dot(da.pdmat4_to_npmat4(mat),da.pdmat4_to_npmat4(self.node_list[i].getMat()))))
    def rot_top(self, angle):
        rot = rm.rotmat_from_axangle((0, 0, 1), angle / 180 * np.pi)
        mat = da.npv3mat3_to_pdmat4((0, 0, 0), rot)
        for i,cube in enumerate(self.node_list):
            if round(cube.getPos()[2], 2)==0.02:
                print(i)
                self.node_list[i].setMat(da.npmat4_to_pdmat4(np.dot(da.pdmat4_to_npmat4(mat),da.pdmat4_to_npmat4(self.node_list[i].getMat()))))

    def rot_rgt(self, angle):
        rot = rm.rotmat_from_axangle((0, 1, 0), angle / 180 * np.pi)
        mat = da.npv3mat3_to_pdmat4((0, 0, 0), rot)
        for i,cube in enumerate(self.node_list):
            # print(round(cube.getPos()[1], 2))
            if round(cube.getPos()[1], 2)==0.02:
                # print(i)
                self.node_list[i].setMat(da.npmat4_to_pdmat4(np.dot(da.pdmat4_to_npmat4(mat),da.pdmat4_to_npmat4(self.node_list[i].getMat()))))
    def rot_lft(self, angle):
        rot = rm.rotmat_from_axangle((0, 1, 0), angle / 180 * np.pi)
        mat = da.npv3mat3_to_pdmat4((0, 0, 0), rot)
        for i,cube in enumerate(self.node_list):
            # print(round(cube.getPos()[1], 2))
            if round(cube.getPos()[1], 2)==-0.02:
                print(i)
                self.node_list[i].setMat(da.npmat4_to_pdmat4(np.dot(da.pdmat4_to_npmat4(mat),da.pdmat4_to_npmat4(self.node_list[i].getMat()))))
    def rot_lftback(self, angle):
        rot = rm.rotmat_from_axangle((1, 0, 0), angle / 180 * np.pi)
        mat = da.npv3mat3_to_pdmat4((0, 0, 0), rot)
        for i,cube in enumerate(self.node_list):
            # print(round(cube.getPos()[1], 2))
            if round(cube.getPos()[0], 2)==0.02:
                print(i)
                self.node_list[i].setMat(da.npmat4_to_pdmat4(np.dot(da.pdmat4_to_npmat4(mat),da.pdmat4_to_npmat4(self.node_list[i].getMat()))))
    def rot_rgtback(self, angle):
        rot = rm.rotmat_from_axangle((1, 0, 0), angle / 180 * np.pi)
        mat = da.npv3mat3_to_pdmat4((0, 0, 0), rot)
        for i,cube in enumerate(self.node_list):
            # print(round(cube.getPos()[1], 2))
            if round(cube.getPos()[0], 2)==-0.02:
                print(i)
                self.node_list[i].setMat(da.npmat4_to_pdmat4(np.dot(da.pdmat4_to_npmat4(mat),da.pdmat4_to_npmat4(self.node_list[i].getMat()))))


    def rot(self, part, angle):
        if part == "lft":
            return self.rot_lft(angle)
        elif part == "rgt":
            return self.rot_rgt(angle)
        elif part == "top":
            return self.rot_top(angle)
        elif part == "bottom":
            return self.rot_bottom(angle)
        elif part == "lftback":
            return self.rot_lftback(angle)
        elif part == "rgtback":
            return self.rot_rgtback(angle)

base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960, h=540, lookat_pos=[0, 0, 0.0])
gm.gen_frame(length=.05, thickness=.0005 ).attach_to(base)
rubik = Rubik()
rubik.attach_to(base.render)
# base.run()
testList = np.linspace(0,90,90)
rot_info = [{"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
{"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
{"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
{"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
{"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
{"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "top", "angle": 90},
            {"type": "rgt", "angle": -90}
            ]

def update(textNode, rubikNode, count, rot_info, task):
    if textNode[0] is not None:
        textNode[0].detachNode()
        textNode[1].detachNode()
        textNode[2].detachNode()
        rubikNode[0].detachNode()
    # if count[0]< 90:
        # rubikNode[0] = NodePath("rubik")
        # rubik.attach_to(rubikNode[0])
        # rubik.rot_rgt(1)
        # count[0]+=1
    if count[1]<len(rot_info):
        rubikNode[0] = NodePath("rubik")
        rubik.attach_to(rubikNode[0])
        if count[0]<abs(rot_info[count[1]]["angle"]):
            rubik.rot(rot_info[count[1]]["type"],rot_info[count[1]]["angle"]/90)
            count[0]+=1
        else:
            count[0]=0
            count[1]+=1

    rubikNode[0].reparentTo(base.render)
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
rubikNode = [None]
count = [0, 0]
taskMgr.doMethodLater(0.001, update, "addobject", extraArgs=[testNode, rubikNode, count, rot_info], appendTask=True)
# a.setPos((0.05,0.05,0.05))
# t22.reparentTo(base.render)
base.run()