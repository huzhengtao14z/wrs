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
import basis.trimesh as trimesh
import trimesh

def show_ikfeasible_poses(obj_rotmat, obj_pos):
    hndfa = rtqhe.RobotiqHE(enable_cc=True)
    obj_fixture = object.copy()
    obj_fixture.set_rotmat(obj_rotmat)
    obj_fixture.set_pos(obj_pos)
    obj_fixture.attach_to(base)
    for i, item in enumerate(grasp_info_list):
        jaw_width, gl_jaw_center_pos, gl_jaw_center_rotmat, hnd_pos, hnd_rotmat = item
        hnd_rotmat = obj_rotmat.dot(gl_jaw_center_rotmat)
        hnd_pos = obj_rotmat.dot(gl_jaw_center_pos) + obj_pos

        hndfa.fix_to(pos=hnd_pos,
                     rotmat=hnd_rotmat, jawwidth=.04)

        # hndfa.gen_meshmodel(rgba=(1, 0, 0, 0.05)).attach_to(base)
        jnt_values = robot.ik(component_name=component_name,
                              tgt_pos=hnd_pos,
                              tgt_rotmat=hnd_rotmat,
                              max_niter=500,
                              toggle_debug=False,
                              seed_jnt_values=None)
        if jnt_values is not None:
            robot.fk(component_name=component_name, jnt_values=jnt_values)

            if hndfa.is_mesh_collided(slopeforcd_high, toggle_debug=False):
                robot.gen_meshmodel(toggle_tcpcs=False, rgba=(1, 0, 0, 0.05)).attach_to(base)
                # pass
            else:
                robot.gen_meshmodel(toggle_tcpcs=False, rgba=(0, 1, 0, 0.5)).attach_to(base)
                print(f"IK Done!, feasible grasp ID {i}")


if __name__ == '__main__':
    # world
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)

    name = "bunnysim.stl"

    # a = trimesh.load("./3dcnnobj/" + name)
    # b = a.voxelized(5).as_boxes()
    # # b.show()
    # b.export(name)
    # base.run()
    # a.show()
    object = cm.CollisionModel(name)
    object.set_scale((.001,.001,.001))
    object.set_rgba([.5, .7, .3, 1])
    gm.gen_frame().attach_to(object)
    object.attach_to(base)
    base.run()

