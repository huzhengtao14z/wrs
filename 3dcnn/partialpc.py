import copy
import math
import visualization.panda.world as wd
import modeling.collision_model as cm
import humath as hm
import hufunc as hf
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
import slope
import Sptpolygoninfo as sinfo
import basis.trimesh as trimeshWan
import trimesh as trimesh
from trimesh.sample import sample_surface
from panda3d.core import NodePath
import trimeshwraper as tw
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.robotiq85.robotiq85 as rtq85
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe
import open3d as o3d
import vision.depth_camera.pcd_data_adapter as vdda
import humath as ma

if __name__ == '__main__':
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)

    # name = "mug.stl"
    # name = "airplaneremesh"
    name = "CoughDropsBerries_800_tex.obj"
    # name = "armadillo.stl"
    # name = "mug"
    obj = tw.TrimeshHu("./kit_model/", name,scale=0.001)
    mesh = obj.outputTrimesh
    testmesh = gm.GeometricModel(mesh)
    testmesh.set_rgba([1, 0, 0, 1])
    # testmesh.attach_to(base)

    origin=np.array([0,0.1,0])

    intersector = trimesh.base.ray.ray_pyembree.RayMeshIntersector(mesh)

    faces = mesh.faces
    vertices = mesh.vertices
    check_list=[]
    origin_list=[]
    for face in faces:
        points = [vertices[face[0]], vertices[face[1]], vertices[face[2]]]
        direction = np.array(hm.centerPoint(points)-origin)
        check_list.append(direction)
        origin_list.append(origin)
    viewed = intersector.intersects_first(ray_origins=origin_list,
                         ray_directions=check_list)
    viewed=hm.listnorepeat(viewed)


    # for item in viewed_faces:
    #     hf.drawanySingleSurface(base, vertices = [vertices[item[0]],vertices[item[1]],vertices[item[2]],hm.centerPoint([vertices[item[0]],vertices[item[1]],vertices[item[2]]])], color=(0,1,0,1))

    viewed_faces = [faces[i] for i in viewed]
    list_viewedvertexid = list(set(np.asarray(viewed_faces).flatten().tolist()))
    def updateid(idlist,face):
        newid0 = idlist.index(face[0])
        newid1 = idlist.index(face[1])
        newid2 = idlist.index(face[2])
        return [newid0,newid1,newid2]

    viewed_vertices = []
    for item in list_viewedvertexid:
            viewed_vertices.append(vertices[item])

    viewed_faces = [updateid(list_viewedvertexid,faces[i]) for i in viewed]
    # for item in viewed_faces:
    #     hf.drawanySingleSurface(base, vertices = [viewed_vertices[item[0]],viewed_vertices[item[1]],viewed_vertices[item[2]],hm.centerPoint([viewed_vertices[item[0]],viewed_vertices[item[1]],viewed_vertices[item[2]]])], color=(0,1,0,1))

    viewedmesh=trimesh.Trimesh(vertices = viewed_vertices, faces=viewed_faces)

    viewedmesh.export("testtest.stl")
    test=gm.GeometricModel("testtest.stl")
    test.set_rgba([1,0,0,1])
    test.attach_to(base)




    def update(textNode, count, task):


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
    taskMgr.doMethodLater(0.01, update, "addobject", extraArgs=[testNode, count],
                          appendTask=True)

    base.run()