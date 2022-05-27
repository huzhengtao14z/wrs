import copy
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
import trimesh as trimesh
from panda3d.core import NodePath

class TrimeshHu(object):
    def __init__(self, meshpath = None, name = None, mesh = None, scale = 1.0):

        if not mesh:
            self.name = name
            self.meshpath = meshpath + name
            self.mesh = trimesh.load(self.meshpath)
        else:
            self.mesh = mesh
        self.__infoUpdate(self.mesh)
        self.originalmesh = trimesh.Trimesh(vertices=self.vertices*scale, faces=self.faces*scale, face_normals=self.face_normals,
                                       vertex_normals=self.vertex_normals)
        self.mesh = copy.copy(self.originalmesh)
        # self.mesh.scale(scale)
    # def set_scale

    def voxelization(self, voxel, hollow):
        self.voxel = voxel
        if hollow == True:
            self.voxelizedmodel = self.mesh.voxelized(voxel).hollow()
            self.tfmatrix = self.voxelizedmodel.transform
            self.matrix = self.voxelizedmodel.matrix
            self.points = self.voxelizedmodel.points
            self.mesh = self.voxelizedmodel.as_boxes()
            # self.mesh = voxelizedmodel.marching_cubes
        else:
            self.voxelizedmodel = self.mesh.voxelized(voxel).fill(method='base')
            self.tfmatrix = self.voxelizedmodel.transform
            self.matrix = self.voxelizedmodel.matrix
            self.points = self.voxelizedmodel.points
            self.mesh = self.voxelizedmodel.as_boxes()
            # self.mesh = voxelizedmodel.marching_cubes
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
            gm.gen_sphere(point, .001, [1, 0, 0, 0.5]).attach_to(base)

    def show_hited_balls(self, observe_origin,target):
        # for i_index, i in enumerate(self.node_matrix):
        #     for j_index, j in enumerate(i):
        #         for k_index, k in enumerate(j):
        #             if self.matrix[i_index][j_index][k_index]:
        #                 gm.gen_sphere(k*0.001, 0.001, [1,0,0,0.5]).attach_to(base)
        # self.hited_list = []
        hited_list =  self.hitray(observe_origin)
        for point in hited_list:
            # gm.gen_sphere(point, .001, [1, 0, 0, 0.5]).attach_to(target)
            gm.gen_box(extent=[self.voxel]*3, homomat = rm.homomat_from_posrot(point), rgba = [1, 0, 0, 1]).attach_to(target)
        return target
    def hitray(self, observe_origin = [.0,.0,-.09]):
        checker = trimesh.base.ray.ray_pyembree.RayMeshIntersector(self.mesh)
        # observation = np.array([[0, 0, .900]])
        observation = np.array([observe_origin])
        ray_directions = [point - observation[0] for point in self.points]
        ray_origins = [observation[0] for point in self.points]
        hitinfo = checker.intersects_id(ray_origins=ray_origins, ray_directions=ray_directions, multiple_hits=False, max_hits=1,
                            return_locations=True)
        hited_list = []
        for i, point in enumerate(self.points):
            if np.linalg.norm(point - hitinfo[2][i]) <= self.voxel:
                hited_list.append(point)
        hited_list = np.asarray(hited_list)
        # self.observed_mesh = self.voxelizedmodel.points_to_indices(self.hited_list)

        return hited_list

    @property
    def outputTrimesh(self):
        self.newmesh = trimeshWan.Trimesh(vertices=self.vertices, faces=self.faces, face_normals=self.face_normals,
                           vertex_normals=self.vertex_normals)
        return self.newmesh

    def __infoUpdate(self, mesh):
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

    name = "bunnysimcal_env.stl"
    mesh = TrimeshHu("./3dcnnobj/", name)

    # name = "bo"
    # box = gm.gen_box([.090,.090,.090]).objtrm

    # mesh = TrimeshHu(mesh = box, scale=1)
    # mesh.set_scale((0.001, 0.001, 0.001))
    # mesh.voxelization(45, hollow = False)

    icosphere = gm.gen_sphere(radius=0.2, rgba =[0,0,1,0.1] ,subdivisions=1)
    sample = icosphere.objtrm.vertices
    # icosphere.attach_to(base)
    # print(sample)

    # import grasping.planning.antipodal as gpa
    # import robot_sim.end_effectors.grippers.robotiq85.robotiq85 as rtq85
    # grasp_info_list = gpa.load_pickle_file('bunnysimcal', './', 'rtq85.pickle')
    # # grasp_info_list = gpa.load_pickle_file('test_long', './', 'rtqhe.pickle')
    # hnd_rotmat = [grasp_info_list[i][4] for i in range(len(grasp_info_list))]
    # gripper_s = rtq85.Robotiq85()
    # for grasp_info in grasp_info_list:
    #     jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
    #     gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    #     gripper_s.gen_meshmodel(rgba=(0.3, 0.3, 0.3, 0.1)).attach_to(base)

    # grasp_info_list = gpa.load_pickle_file('bunysim', './', 'rtq85.pickle')
    # # grasp_info_list = gpa.load_pickle_file('test_long', './', 'rtqhe.pickle')
    # hnd_rotmat = [grasp_info_list[i][4] for i in range(len(grasp_info_list))]
    # gripper_s = rtq85.Robotiq85()
    # for grasp_info in grasp_info_list:
    #     jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
    #     gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    #     gripper_s.gen_meshmodel(rgba=(0, 1, 0, 0.03)).attach_to(base)



    mesh.meshTransform(rotaxis = np.array([0,0,1]), angle = np.radians(0), translation=np.array([0,0,0]))
    mesh.voxelization(.0045, hollow = True)
    mesh.get_node_matrix()
    mesh.get_transform()
    # mesh.show_balls()
    # mesh.show_hited_balls(base)
    # mesh.export(this_dir,"box_vox")
    c = cm.CollisionModel(mesh.outputTrimesh)
    # c.set_scale((0.001, 0.001, 0.001))
    c.set_rgba([0, 191 / 255, 1, 0.8])
    c.attach_to(base)
    # mesh.show_hited_balls(observe_origin=sample[5], target=base.render)
    base.run()

    objNode = [None]
    voxelNode = [None]
    observeNode = [None]


    def update(textNode, objNode, voxelNode, observeNode, count, task):
        if observeNode[0] is not None:
            observeNode[0].detachNode()
        observeNode[0] = NodePath("observe")
        mesh.show_hited_balls(observe_origin=sample[count[0]],target = observeNode[0])
        gm.gen_sphere(sample[count[0]]).attach_to(observeNode[0])
        observeNode[0].reparent_to(base.render)
        count[0]+=1

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
    count = [0]
    taskMgr.doMethodLater(1,update,  "addobject", extraArgs=[testNode, objNode, voxelNode, observeNode, count], appendTask=True)




    # print(b)
    base.run()

