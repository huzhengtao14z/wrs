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
import robot_sim.robots.xarm7_shuidi_mobile.xarm7_shuidi_mobile as xarm
import motion.probabilistic.rrt_connect as rrtc
import hupackage.hupackage as hu

if __name__ == '__main__':
    base = wd.World(cam_pos=[2, 1, 3], w=960,
                    h=540, lookat_pos=[0, 0, 1.1])
    gm.gen_frame().attach_to(base)

    object = cm.CollisionModel("./objects/tubebig.stl")
    object.set_pos(np.array([0.800, -.500, 0.700]))
    object.set_rgba([.5, .7, .3, 1])
    object.attach_to(base)
    component_name = 'rgt_arm'
    # robot_instance = ur3d.UR3Dual()
    # robot_instance = ur3ed.UR3EDual()
    robot_instance = sda5.SDA5F()
    # robot_instance = xarm.XArm7YunjiMobile()

    start_hnd_pos = np.array([0.6, -0.5, 0.750])
    start_hnd_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi / 2)
    goal_hnd_pos = np.array([0.8, -0.601, 1.0])
    goal_hnd_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi / 2)
    start_jntsangle = robot_instance.ik(component_name, start_hnd_pos, start_hnd_rotmat)
    goal_jntsangle  = robot_instance.ik(component_name, goal_hnd_pos,  goal_hnd_rotmat)

    hu.debugpos(start_hnd_pos, start_hnd_rotmat, base)
    robot_instance.fk(component_name, start_jntsangle)
    # gm.gen_frame(pos=start_hnd_pos, rotmat=start_hnd_rotmat).attach_to(base)
    # gm.gen_frame(pos=goal_hnd_pos, rotmat=goal_hnd_rotmat).attach_to(base)
    robot_meshmodel = robot_instance.gen_meshmodel(tcp_loc_pos=start_hnd_pos, tcp_loc_rotmat=start_hnd_rotmat)
    # robot_instance.gen_meshmodel()
    # robot_instance.fk(component_name, goal_jntsangle)
    # robot_instance.gen_meshmodel()
    robot_meshmodel.attach_to(base)
    base.run()
    rrtc_planner = rrtc.RRTConnect(robot_instance)
    path = rrtc_planner.plan(start_conf=start_jntsangle,
                             goal_conf=goal_jntsangle,
                             obstacle_list=[object],
                             ext_dist=.1,
                             max_time=300,
                             component_name=component_name)

    for pose in path:
        # print(pose)
        robot_instance.fk(component_name, pose)
        # robot_meshmodel = robot_instance.gen_meshmodel()
        robot_instance.gen_meshmodel().attach_to(base)
        # robot_meshmodel.attach_to(base)
        # robot_meshmodel.show_cdprimit()
        # robot_instance.gen_stickmodel().attach_to(base)


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