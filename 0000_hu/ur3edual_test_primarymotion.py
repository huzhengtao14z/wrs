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
import pickle
import hupackage.motionplanning as huplanner
import manipulation.pick_place_planner as ppp

if __name__ == '__main__':
    base = wd.World(cam_pos=[2, 1, 3], w=960,
                    h=540, lookat_pos=[0, 0, 1.1])
    gm.gen_frame().attach_to(base)

    object = cm.CollisionModel("./objects/tubebig.stl")
    # object.set_scale([0.001, 0.001, 0.001])
    object.set_pos(np.array([1.05, -.3, 1.3]))
    object.set_rgba([.5, .7, .3, 1])
    # object.attach_to(base)
    object_start = object.copy()
    object_start_pos = np.array([0.800, -.350, 0.900])
    object_start_rotmat = np.eye(3)
    object_start_homomat = rm.homomat_from_posrot(object_start_pos, object_start_rotmat)
    object_start.set_pos(object_start_pos)
    object_start.set_rotmat(object_start_rotmat)
    object_start.attach_to(base)

    object_goal = object.copy()
    object_goal_pos = np.array([0.800, -.300, 0.900])
    object_goal_rotmat = np.eye(3)
    object_goal_homomat = rm.homomat_from_posrot(object_goal_pos, object_goal_rotmat)
    object_goal.set_pos(object_goal_pos)
    object_goal.set_rotmat(object_goal_rotmat)
    object_goal.attach_to(base)



    component_name = 'rgt_arm'
    # robot_instance = ur3d.UR3Dual()
    robot_instance = ur3ed.UR3EDual()
    # robot_instance = sda5.SDA5F()
    # robot_instance.gen_meshmodel().attach_to(base)
    start_tcp_pos, start_tcp_rotmat = robot_instance.get_init_tcp(component_name)
    print(robot_instance.get_jnt_values(component_name))
    goal_tcp_pos = start_tcp_pos + np.array([0,-0.000,-0.005])
    goal_tcp_rotmat = start_tcp_rotmat

    grasp_info_list = gpa.load_pickle_file('tubebig', './', 'graspdata/hnde_tubebig.pickle')

    # planner = huplanner.PrimaryPlanner(robot_instance)
    # path = planner.gen_linear_moveto( component_name,
    #                                   start_tcp_pos,
    #                                   start_tcp_rotmat,
    #                                   goal_tcp_pos,
    #                                   goal_tcp_rotmat,
    #                                   obstacle_list=[])
    ppp = ppp.PickPlacePlanner(robot_instance)
    goal_homomat_list = [object_start_homomat, object_goal_homomat]
    for grasp_id in range(len(grasp_info_list)):
        grasp_info = grasp_info_list[grasp_id]
        jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
        conf_list, jawwidthlist, objpose_list = ppp.gen_holding_moveto(hand_name = component_name,
                                   objcm = object,
                                   grasp_info = grasp_info,
                                   obj_pose_list = goal_homomat_list,
                                   approach_direction_list=[None, np.array([0, 0, -1]), None],
                                   approach_distance_list=[.05] * 3,
                                   depart_direction_list=[np.array([0, 0, 1]), None, None],
                                   depart_distance_list=[.05] * 3,
                                   ad_granularity=.007,
                                   use_rrt=True,
                                   obstacle_list=[],
                                   seed_jnt_values=None)
        if conf_list is not None:
            break
    path = conf_list
    objpose =objpose_list
    print(objpose)
    for i,pose in enumerate(path):
        # print(pose)
        robot_instance.fk(component_name, pose)
        # robot_meshmodel = robot_instance.gen_meshmodel()
        # robot_instance.gen_meshmodel().attach_to(base)
        object_show = object.copy()
        object_show.set_homomat(objpose[i])
        # object_show.attach_to(base)
        # robot_meshmodel.attach_to(base)
        # robot_meshmodel.show_cdprimit()
        # robot_instance.gen_stickmodel().attach_to(base)


    def update(robot,
               object_box,
               robot_path,
               jawwidth_path,
               obj_path,
               robot_attached_list,
               object_attached_list,
               counter, textNode,
               task):
        if counter[0] >= len(robot_path):
            counter[0] = 0
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            for object_attached in object_attached_list:
                object_attached.detach()
            robot_attached_list.clear()
            object_attached_list.clear()
        pose = robot_path[counter[0]]
        robot.fk(component_name, pose)
        robot.jaw_to(component_name, jawwidth_path[counter[0]])
        robot_meshmodel = robot.gen_meshmodel()
        robot_meshmodel.attach_to(base)
        # print()
        robot_attached_list.append(robot_meshmodel)
        obj_pose = obj_path[counter[0]]
        objb_copy = object_box.copy()
        objb_copy.set_rgba([1, 0, 0, 1])
        objb_copy.set_homomat(obj_pose)
        objb_copy.attach_to(base)
        object_attached_list.append(objb_copy)
        counter[0] += 1

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
    robot_attached_list = []
    object_attached_list = []
    counter = [0]
    textNode = [None, None, None]
    jawwidth_list = jawwidthlist
    taskMgr.doMethodLater(0.05, update, "update",
                          extraArgs=[robot_instance,
                                     object,
                                     conf_list,
                                     jawwidth_list,
                                     objpose_list,
                                     robot_attached_list,
                                     object_attached_list,
                                     counter, textNode],
                          appendTask=True)



    # taskMgr.add(update, "addobject", extraArgs=[testNode], appendTask=True)
    base.run()