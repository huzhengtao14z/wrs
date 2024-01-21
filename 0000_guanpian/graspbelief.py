import copy
import math
import visualization.panda.world as wd
import modeling.collision_model as cm
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
# import slope
# import Sptpolygoninfo as sinfo
import basis.trimesh as trimeshWan
import trimesh as trimesh
from panda3d.core import NodePath
import trimeshwraper as tw
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.robotiq85.robotiq85 as rtq85
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe
import open3d as o3d
import vision.depth_camera.pcd_data_adapter as vdda

if __name__ == '__main__':
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)

    # name = "mug"
    # name = "airplaneremesh"
    # name = "guanpian"
    # name = "armadillo"
    name = "mug_s"
    mesh = tw.TrimeshHu("./objects/", name+".stl", scale=1)


    # icosphere = gm.gen_sphere(radius=200, rgba=[0, 0, 1, 0.1], subdivisions=2)
    # sample = icosphere.objtrm.vertices
    # icosphere.attach_to(base)


    mesh.meshTransform(rotaxis=np.array([0, 0, 1]), angle=np.radians(0), translation=np.array([0, 0, 0]))
    # mesh.voxelization(0.0015, hollow=True) #"muster"
    # mesh.voxelization(0.001, hollow=True) #"bunnysim04"
    mesh.voxelization(0.01, hollow=True)  # ""
    # mesh.get_node_matrix()
    # mesh.get_transform()
    # mesh.show_balls()
    # guanpian = gm.GeometricModel(mesh)
    # guanpian.attach_to(base)


    # mesh.show_hited_balls(observe_origin=(-1, 0, 0), target=base, shape = "box", generateCKlist=False)
    base.run()



    grasp_info_list = gpa.load_pickle_file(name, './', 'grasp/hande.pickle')
    hnd_rotmat = [grasp_info_list[i][4] for i in range(len(grasp_info_list))]
    gripper_s = rtqhe.RobotiqHE()


    mesh.cpt_briefgrasp(observe_origin=(-1, 0, 0), target=base, gripper=gripper_s, grasp_info_list=grasp_info_list)
    # mesh.cpt_briefgrasp(observe_origin=(-0.9, 0, 0.9), target=base, gripper=gripper_s, grasp_info_list=grasp_info_list)
    # mesh.cpt_briefgrasp(observe_origin=(-0.9, 0.90, 0.9), target=base, gripper=gripper_s, grasp_info_list=grasp_info_list)
    # mesh.cpt_briefgrasp(observe_origin=(0.9, -0.90, 0.9), target=base, gripper=gripper_s,
    #                     grasp_info_list=grasp_info_list)
    # mesh.export(this_dir, "box_vox")
    c = cm.CollisionModel(mesh.outputTrimesh)
    # c.set_scale((0.001, 0.001, 0.001))
    c.set_rgba((0, 1, 0, .11))
    c.attach_to(base)



    base.run()





    objNode = [None]
    voxelNode = [None]
    observeNode = [None]


    def update(textNode, objNode, voxelNode, observeNode, count, task):
        if observeNode[0] is not None:
            observeNode[0].detachNode()
        observeNode[0] = NodePath("observe")
        mesh.show_hited_balls(observe_origin=sample[count[0]], target=observeNode[0])
        gm.gen_sphere(sample[count[0]]).attach_to(observeNode[0])
        observeNode[0].reparent_to(base.render)
        count[0] += 1

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
    taskMgr.doMethodLater(1, update, "addobject", extraArgs=[testNode, objNode, voxelNode, observeNode, count],
                          appendTask=True)

    base.run()