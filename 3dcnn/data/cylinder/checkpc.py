import copy
import math
import random

from keras.models import Sequential, Model, load_model
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
# import open3d.geometry as o3dg
import vision.depth_camera.pcd_data_adapter as vdda


def randfloat(num, l, h):
    if l > h:
        return None
    else:
        a = h - l
        b = h - a
        out = (np.random.rand(num) * a + b).tolist()
        # out = np.random.randint(num) * a + b
        out = np.array(out)
        return out

if __name__ == '__main__':
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    # gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)

    def generatemodel(type,**kwargs):
        cata = type
        if cata == 'cylinder':
            number = kwargs['number']
            heightrange = kwargs['heightrange']
            diameterrange = kwargs['diameterrange']
            heightlist = np.random.randint(heightrange[0],heightrange[1],size = [number])
            diameterlist = np.random.randint(diameterrange[0],diameterrange[1],size = [number])

            for height in heightlist:
                for diameter in diameterlist:
                    name = f'{cata}-h{str(height)}d{str(diameter)}.stl'
                    scale = 0.001
                    model = gm.gen_cylinder(diameter*0.5*scale, height*scale)
                    model_tri = model.objtrm
                    model_tri.export(name)
        if cata == 'cuboid':
            number = kwargs['number']
            heightrange = kwargs['heightrange']
            widthrange = kwargs['widthrange']
            lengthrange = kwargs['lengthrange']
            heightlist = np.random.randint(heightrange[0], heightrange[1], size=[number])
            widthlist = np.random.randint(widthrange[0], widthrange[1], size=[number])
            lengthlist = np.random.randint(lengthrange[0], lengthrange[1], size=[number])

            for height in heightlist:
                for width in widthlist:
                    for length in lengthlist:
                        name = f'{cata}-h{str(height)}w{str(width)}l{str(length)}.stl'
                        scale = 0.001
                        model = gm.gen_box(extent=[height*scale, width*scale, length*scale])
                        model_tri = model.objtrm
                        model_tri.export(name)


    generatemodel('cylinder', heightrange=[20,200], diameterrange = [20,60], number = 3)
    generatemodel('cuboid', heightrange=[20, 200], widthrange=[20, 60], lengthrange=[10, 100], number=3)

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