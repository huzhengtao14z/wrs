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
import manipulation.pick_place_planner as ppp
import os
import pickle
import basis.data_adapter as da
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe
import slope
import Sptpolygoninfo as sinfo
import basis.trimesh as trimeshWan
import trimesh

class TrimeshHu(object):
    def __init__(self, meshpath = None, name = None, mesh = None):

        if not mesh:
            self.name = name
            self.meshpath = meshpath + name
            self.mesh = trimesh.load(self.meshpath)
        else:
            self.mesh = mesh
        self.__infoUpdate(self.mesh)
        self.mesh = trimesh.Trimesh(vertices=self.vertices, faces=self.faces, face_normals=self.face_normals,
                                       vertex_normals=self.vertex_normals)

    # def set_scale

    def voxelization(self, voxel, hollow):
        self.voxel = voxel
        if hollow == True:
            voxelizedmodel = self.mesh.voxelized(voxel).hollow()
            self.tfmatrix = voxelizedmodel.transform
            self.matrix = voxelizedmodel.matrix
            self.points = voxelizedmodel.points
            self.mesh = voxelizedmodel.as_boxes()
        else:
            voxelizedmodel = self.mesh.voxelized(voxel).fill(method='base')
            self.tfmatrix = voxelizedmodel.transform
            self.matrix = voxelizedmodel.matrix
            self.points = voxelizedmodel.points
            self.mesh = voxelizedmodel.as_boxes()
        self.__infoUpdate(self.mesh)

    def get_node_matrix(self):
        matrix = [[[[self.matrix[i][j][k]*i*self.voxel+self.tfmatrix[0][3], self.matrix[i][j][k]*j*self.voxel+self.tfmatrix[1][3], self.matrix[i][j][k]*k*self.voxel+self.tfmatrix[2][3]] for k in range(len(self.matrix[i][j]))] for j in range(len(self.matrix[i]))] for i in range(len(self.matrix))]
        # print(np.asarray(matrix, dtype=float))
        self.node_matrix = np.asarray(matrix)
        print(self.node_matrix)
        return self.node_matrix

    def get_transform(self):
        print(self.tfmatrix)
        return self.tfmatrix

    def show_balls(self):
        # for i_index, i in enumerate(self.node_matrix):
        #     for j_index, j in enumerate(i):
        #         for k_index, k in enumerate(j):
        #             if self.matrix[i_index][j_index][k_index]:
        #                 gm.gen_sphere(k*0.001, 0.001, [1,0,0,0.5]).attach_to(base)

        for point in self.points:
            gm.gen_sphere(point * 0.001, 0.001, [1, 0, 0, 0.5]).attach_to(base)
        # return self.tfmatrix

    @property
    def outputTrimesh(self):
        self.newmesh = trimeshWan.Trimesh(vertices=self.vertices, faces=self.faces, face_normals=self.face_normals,
                           vertex_normals=self.vertex_normals)
        return self.newmesh

    def __infoUpdate(self, mesh):
        # self.faces = mesh.faces
        # self.vertices = mesh.vertices
        # self.face_normals = mesh.face_normals
        # self.vertex_normals = mesh.vertex_normals
        self.faces = np.asarray(mesh.faces)
        self.vertices = np.asarray(mesh.vertices)
        self.face_normals = np.asarray(mesh.face_normals)
        self.vertex_normals = np.asarray(mesh.vertex_normals)

    def meshTransform(self, rotaxis = np.array([0,0,1]), angle = np.radians(90), translation=np.array([0,0,0])):
        rotmat = rm.rotmat_from_axangle(rotaxis, angle)
        homomate = rm.homomat_from_posrot(translation, rotmat)
        self.mesh.apply_transform(homomate)
        # self.vertices = np.asarray([rotmat.dot(vert) + translation for vert in self.mesh.vertices])
        # self.faces = self.mesh.faces
        # self.face_normals =  np.asarray([rotmat.dot(face_normal) + translation for face_normal in self.mesh.face_normals])
        # self.vertex_normals =  np.asarray([rotmat.dot(vertex_normal) + translation for vertex_normal in self.mesh.vertex_normals])
        # self.mesh = trimesh.Trimesh(vertices=self.vertices, faces=self.faces, face_normals=self.face_normals,
        #                                vertex_normals=self.vertex_normals)

    def export(self, outputfile, outputname):
        this_dir, this_filename = os.path.split(__file__)
        extention = ".stl"
        if outputfile:
            outputfile = outputfile
        else:
            outputfile = this_dir
        self.mesh.export(outputfile + "/" + outputname + extention)

if __name__ == '__main__':

    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)

    # name = "bunnysim.stl"
    name = "bo"
    box = gm.gen_box([90,90,90]).objtrm

    # a = trimesh.load("./3dcnnobj/" + name)

    # mesh = TrimeshHu("./3dcnnobj/", name)
    mesh = TrimeshHu(mesh = box)
    # mesh.set_scale((0.001, 0.001, 0.001))
    # mesh.voxelization(45, hollow = False)
    mesh.meshTransform(rotaxis = np.array([0,0,1]), angle = np.radians(45), translation=np.array([0,0,0]))
    mesh.voxelization(10, hollow = True)
    mesh.get_node_matrix()
    mesh.get_transform()
    mesh.show_balls()
    mesh.export(this_dir,"box_vox")
    c = cm.CollisionModel(mesh.outputTrimesh)
    c.set_scale((0.001, 0.001, 0.001))
    c.set_rgba((0,1,0,0.052))
    c.attach_to(base)
    base.run()

