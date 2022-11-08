import robot_sim.robots.ur3e_dual.ur3e_dual as ur3ed
import modeling.dynamics.bullet.bdmodel as bdm
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import basis.robot_math as rm
import numpy as np
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as hande
import basis.data_adapter as da
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletSliderConstraint
from panda3d.core import *

import modeling.dynamics.pybullet
import math


def get_lnk_bdmodel(robot_s, component_name, lnk_id):
    lnk = robot_s.manipulator_dict[component_name].lnks[lnk_id]
    bd_lnk = bdm.BDModel(lnk["collisionmodel"], mass=0, type="box", friction=0.001, dynamic=False)
    bd_lnk.set_homomat(rm.homomat_from_posrot(lnk["gl_pos"], lnk["gl_rotmat"]))
    return bd_lnk

def get_lnk_gri_bdmodel(gripper):
    lft_lnk = gripper.lft.lnks[1]
    # lft_bd_lnk = lft_lnk["dynamicmodel"]
    # lft_bd_lnk.set_homomat(rm.homomat_from_posrot(lft_lnk["gl_pos"], lft_lnk["gl_rotmat"]))
    lft_bd_lnk =  bdm.BDModel(lft_lnk["meshfile"], mass=10000, type="box", friction=0.0000, dynamic=True)
    lft_bd_lnk.set_homomat(rm.homomat_from_posrot(lft_lnk["gl_pos"], lft_lnk["gl_rotmat"]))

    lft_bd_lnk.set_linearVelocity(da.npv3_to_pdv3(np.array([5,0,0])))



    lft_base = gripper.lft.lnks[0]
    lft_bd_base = lft_base["dynamicmodel"]
    lft_bd_base.set_homomat(rm.homomat_from_posrot(lft_base["gl_pos"], lft_base["gl_rotmat"]))

    rgt_lnk = gripper.rgt.lnks[1]
    # rgt_bd_lnk = rgt_lnk["dynamicmodel"]
    rgt_bd_lnk = bdm.BDModel(rgt_lnk["meshfile"], mass=10000, type="box", friction=0.0000,dynamic=True)
    rgt_bd_lnk.set_homomat(rm.homomat_from_posrot(rgt_lnk["gl_pos"], rgt_lnk["gl_rotmat"]))
    # rgt_bd_lnk.set_linearVelocity(da.npv3_to_pdv3(np.array([50, 0, 0])))

    frameA = TransformState.makeMat(da.npmat4_to_pdmat4(rm.homomat_from_posrot(rgt_lnk["gl_pos"], rgt_lnk["gl_rotmat"])))
    frameA = TransformState.makePosHpr(Point3(0,0,0), Vec3(0,0,0))
    # frameB = TransformState.makeMat(da.npmat4_to_pdmat4(rm.homomat_from_posrot(lft_base["gl_pos"], lft_base["gl_rotmat"])))
    cs = BulletSliderConstraint(rgt_bd_lnk.bdb,  frameA, True)
    cs.setDebugDrawSize(2.0)
    cs.setLowerLinearLimit(-100)
    cs.setUpperLinearLimit(100)
    print(da.npmat4_to_pdmat4(rm.homomat_from_posrot(rgt_lnk["gl_pos"], rgt_lnk["gl_rotmat"])))
    base.physicsworld.attachConstraint(cs)

    frameA = TransformState.makeMat(
        da.npmat4_to_pdmat4(rm.homomat_from_posrot(rgt_lnk["gl_pos"], rgt_lnk["gl_rotmat"])))
    frameA = TransformState.makePosHpr(Point3(0, 0, 0), Vec3(0, 0, 0))
    # frameB = TransformState.makeMat(da.npmat4_to_pdmat4(rm.homomat_from_posrot(lft_base["gl_pos"], lft_base["gl_rotmat"])))
    cs = BulletSliderConstraint(lft_bd_lnk.bdb, frameA, True)
    cs.setDebugDrawSize(2.0)
    cs.setLowerLinearLimit(-100)
    cs.setUpperLinearLimit(100)
    print(da.npmat4_to_pdmat4(rm.homomat_from_posrot(rgt_lnk["gl_pos"], rgt_lnk["gl_rotmat"])))
    base.physicsworld.attachConstraint(cs)


    rgt_bd_lnk.set_linearVelocity(da.npv3_to_pdv3(np.array([-5, 0, 0])))


    return lft_bd_lnk, rgt_bd_lnk, lft_bd_base

def update_robot_bdmodel(robot_s, bd_lnk_list):
    cnter = 0
    for arm_name in ["lft_arm", "rgt_arm"]:
        for lnk_id in [1, 2, 3, 4, 5, 6]:
            lnk = robot_s.manipulator_dict[arm_name].lnks[lnk_id]
            bd_lnk_list[cnter].set_homomat(rm.homomat_from_posrot(lnk["gl_pos"], lnk["gl_rotmat"]))
            cnter+=1

def update_gripper_bdmodel(gripper):
    lft_lnk = gripper.lft.lnks[1]
    lft_bd_lnk = lft_lnk["dynamicmodel"]
    lft_bd_lnk.set_homomat(rm.homomat_from_posrot(lft_lnk["gl_pos"], lft_lnk["gl_rotmat"]))

    lft_base = gripper.lft.lnks[0]
    lft_bd_base = lft_base["dynamicmodel"]
    lft_bd_base.set_homomat(rm.homomat_from_posrot(lft_base["gl_pos"], lft_base["gl_rotmat"]))

    rgt_lnk = gripper.rgt.lnks[1]
    rgt_bd_lnk = rgt_lnk["dynamicmodel"]
    rgt_bd_lnk.set_homomat(rm.homomat_from_posrot(rgt_lnk["gl_pos"], rgt_lnk["gl_rotmat"]))


    return lft_bd_lnk, rgt_bd_lnk, lft_bd_base

def get_gripper_bdmoel(gripper):
    bd_lnk_list = get_lnk_gri_bdmodel(gripper)
    # for lnk_id in [0,1]:
    #     bd_lnk_list.append(get_lnk_gri_bdmodel(gripper, lnk_id))
    return bd_lnk_list


if __name__ == '__main__':
    import os

    base = wd.World(cam_pos=[0, 0, 1], lookat_pos=[0, 0, 0])
    base.setFrameRateMeter(True)
    gm.gen_frame().attach_to(base)
    obj_box = cm.gen_box(extent=[.02, 0.01, .03], rgba=[.3, 0, 0, 1])
    # obj_box = cm.gen_sphere(radius=.5, rgba=[.3, 0, 0, 1])
    obj_bd_box = bdm.BDModel(obj_box, mass=3, friction=0.00001 , type="triangles")
    obj_bd_box.set_pos(np.array([0.005, 0, 0.3]))

    obj_bd_box.set_linearVelocity(da.npv3_to_pdv3(np.asarray([0, 0, 0])))
    # obj_bd_box.set_linearVelocity(da.npv3_to_pdv3(np.asarray([0, 10, 0])))

    obj_bd_box.start_physics()
    base.attach_internal_update_obj(obj_bd_box)

    gripper = hande.RobotiqHE(enable_cc=True, dynamic=True)
    gripper.jaw_to(0.05)
    # robot_s.gen_stickmodel().attach_to(base)
    # gripper.show_cdprimit()
    # gripper.dynamic_jaw()
    bd_lnk_list = get_gripper_bdmoel(gripper)
    for bdl in bd_lnk_list:
        bdl.start_physics()
        base.attach_internal_update_obj(bdl)


    base.run()
    def update(gripper, bd_lnk_list, task):
        print(obj_bd_box.get_linearVelocity())
        if base.inputmgr.keymap['space'] is True:

            jaw_values = gripper.get_jawwidth()
            jaw_values_step = 0.001
            jaw_values_new=jaw_values - jaw_values_step
            gripper.jaw_to(jaw_values_new)
            gripper.show_cdprimit()
            update_gripper_bdmodel(gripper)
            base.inputmgr.keymap['space'] = False

        return task.cont
    taskMgr.add(update, "update", extraArgs=[gripper, bd_lnk_list], appendTask=True)
    base.run()
