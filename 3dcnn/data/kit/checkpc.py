import copy
import math
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



if __name__ == '__main__':
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    # gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)
    plydir = 'C:/Users/GL65/Documents/GitHub/wrs/3dcnn/data/kit/pc_500'
    # name = "Amicelli_800_tex"
    namelist = os.listdir(plydir)
    name =  namelist[603].split('.')[0]
    # n = str(1)
    # pcd = o3d.io.read_point_cloud('./pairtial/pairtial_pc/'+name+n+".ply")

    # pcd = o3d.io.read_point_cloud('./pc_500/' + name + ".ply")
    # pcd.paint_uniform_color([0,0,0.5])
    # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # normals = np.asarray(pcd.normals)
    # print(normals)

    # o3d.visualization.draw_geometries([pcd])

    # p_bunny = trimesh.Trimesh()
    mesh = o3d.io.read_triangle_mesh('test_bunny.stl')
    pcd = mesh.sample_points_poisson_disk(500)

    model = load_model('fcn-com.h5')
    pcd_list = vdda.o3dpcd_to_parray(pcd)
    result = model.predict(np.expand_dims(pcd_list, axis=0),  verbose=1)

    with open('pairtial/com_v2.pickle', 'rb') as f:
        com_list = pickle.load(f)
    com = com_list[name]
    # gm.gen_sphere(com, radius=0.003).attach_to(base)
    gm.gen_pointcloud(pcd_list).attach_to(base)
    gm.gen_sphere(result[0], radius=0.003, rgba=(0,0,1,1)).attach_to(base)
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