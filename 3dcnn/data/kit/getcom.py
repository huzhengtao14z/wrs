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
# import open3d.geometry as o3dg
import vision.depth_camera.pcd_data_adapter as vdda



if __name__ == '__main__':
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)
    objmodeldir = 'C:/Users/GL65/Documents/GitHub/wrs/3dcnn/kit_model'
    namelist = os.listdir(objmodeldir)
    comdir = {}
    for name in namelist:
        objmesh = cm.CollisionModel(objmodeldir+'/'+name)
        com = objmesh.get_com()
        objname = name.split('.')[0]
        for n in range(0,12):
            comdir[objname+str(n)]=com

    n = str(10)
    name = "Amicelli_800_tex"
    pcd = o3d.io.read_point_cloud('./pairtial/pairtial_pc/'+name+n+".ply")
    pcd.paint_uniform_color([0,0,0.5])
    pcd_list = vdda.o3dpcd_to_parray(pcd)
    with open('pairtial/comdir.pickle', 'wb') as f:
        pickle.dump(comdir, f)
    com = comdir[name+n]
    gm.gen_sphere(com).attach_to(base)
    gm.gen_pointcloud(pcd_list).attach_to(base)
    base.run()


    def apply_noise(pcd, mu, sigma):
        noisy_pcd = copy.deepcopy(pcd)
        points = np.asarray(noisy_pcd.points)
        points += np.random.normal(mu, sigma, size=points.shape)
        noisy_pcd.points = o3d.utility.Vector3dVector(points)
        return noisy_pcd

    mu, sigma = 0, 0.001  # mean and standard deviation
    source_noisy = apply_noise(pcd, mu, sigma)
    o3d.visualization.draw_geometries([source_noisy])
    print("Source PointCloud + noise:")

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