import copy
import math
import visualization.panda.world as wd
import modeling.collision_model as cm
import humath as hm
# import hufunc as hf
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
from trimesh.sample import sample_surface
from panda3d.core import NodePath
import trimeshwraper as tw
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.robotiq85.robotiq85 as rtq85
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe
import open3d as o3d
import vision.depth_camera.pcd_data_adapter as vdda
import basis.o3dhelper as o3dhelper
import humath as ma

from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import find_and_connect, confirm_capture_3d


class CapturePointCloud(object):
    def __init__(self):
        self.camera = Camera()
        self.frame_all_2d_3d = Frame2DAnd3D()

    def capture_point_cloud(self):
        point_cloud_file = "PointCloud.ply"
        show_error(
            self.frame_all_2d_3d.frame_3d().save_untextured_point_cloud(FileFormat_PLY, point_cloud_file))
        print("Capture and save the untextured point cloud: {}.".format(
            point_cloud_file))

    def capture_textured_point_cloud(self):
        textured_point_cloud_file = "TexturedPointCloud.ply"
        show_error(self.frame_all_2d_3d.save_textured_point_cloud(FileFormat_PLY,
                                                                  textured_point_cloud_file))
        print("Capture and save the textured point cloud: {}".format(
            textured_point_cloud_file))

    def main(self):
        if find_and_connect(self.camera):
            if not confirm_capture_3d():
                return
            show_error(self.camera.capture_2d_and_3d(self.frame_all_2d_3d))
            self.capture_point_cloud()
            self.capture_textured_point_cloud()
            self.camera.disconnect()
            print("Disconnected from the camera successfully.")

        pcd = o3d.io.read_point_cloud('PointCloud.ply')
        # pcd = pcd.uniform_down_sample(5000)

        o3d.visualization.draw_geometries([pcd],
                                          zoom=1,
                                          front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532],
                                          up=[-0.0694, -0.9768, 0.2024])

        pcd_np = vdda.o3dpcd_to_parray(pcd)
        center = np.average(pcd_np, axis=0)
        pcd.translate(-center)
        pcd_np = vdda.o3dpcd_to_parray(pcd)

        gm.GeometricModel(pcd_np).attach_to(base)
        base.run()
if __name__ == '__main__':
    # base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
    #                 h=540, lookat_pos=[0, 0, 0])
    base = wd.World(cam_pos=[2015.57, 637.317, 1881.33], w=3960,
                    h=2540, lookat_pos=[0, 0, 1])
    gm.gen_frame(length=100,
              thickness=5,).attach_to(base)
    this_dir, this_filename = os.path.split(__file__)

    mm = CapturePointCloud()
    mm.main()


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