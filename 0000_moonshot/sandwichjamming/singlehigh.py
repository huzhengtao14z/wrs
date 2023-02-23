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



if __name__ == "__main__":
    base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960, h=540, lookat_pos=[0, 0, 0.0])
    gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)

    interval = 0.006
    # len_num = 39
    # wid_num = 39
    len_num =9
    wid_num = 9
    n = 32
    r = 0.04
    height_layer = 32
    matrix = [[np.array([interval * x, interval * y, 0.000]) for x in range(len_num)] for y in range(wid_num)]
    matrix_second = [[np.array([interval * x, interval * y, 2*interval]) for x in range(len_num)] for y in range(wid_num)]
    cut_list = []

    grid = Grid(np.array(matrix), interval)
    node = Node(grid, height=0.006, origin_offset=0.002)
    matrix_infos = node.node_matrix_infos
    for key in matrix_infos.keys():
        element = Element(matrix_infos[key], dict="thin",radius=0.0006, id = key, cut = cut_list, support = False, secondlayer = False)

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