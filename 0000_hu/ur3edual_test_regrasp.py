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

if __name__ == '__main__':
    base = wd.World(cam_pos=[2, 1, 3], w=960,
                    h=540, lookat_pos=[0, 0, 1.1])
    gm.gen_frame().attach_to(base)

    object = cm.CollisionModel("./objects/lshape-30.stl")
    object.set_scale([0.001, 0.001, 0.001])
    object.set_pos(np.array([1.05, -.3, 1.3]))
    object.set_rgba([.5, .7, .3, 1])
    object.attach_to(base)
    component_name = 'rgt_arm'
    # robot_instance = ur3d.UR3Dual()
    robot_instance = ur3ed.UR3EDual()
    # robot_instance = sda5.SDA5F()
    # robot_instance.gen_meshmodel().attach_to(base)

    with open("path_test.pickle", "rb") as file:
        objmsmp, numikrmsmp, jawwidthmp, originalpathnidlist = pickle.load(file)

    keyposes = []
    # for path in numikrmsmp:
    #     robot_instance.fk("rgt_arm",path[0][1])
    #     start_rgt = robot_instance.get_gl_tcp("rgt_arm")
    #     start_lft = robot_instance.fk("lft_arm", path[0][2])
    #     goal_rgt = robot_instance.fk("rgt_arm",path[-1][1])
    #     goal_lft = robot_instance.fk("lft_arm", path[-1][2])
    #     keyposes.append([start_rgt, start_lft, goal_rgt, goal_lft])

    # robot_instance.fk("rgt_arm", numikrmsmp[4][0][1]*np.pi/180)
    # start_config_rgt = robot_instance.get_gl_tcp("rgt_arm")
    #
    # robot_instance.fk("rgt_arm", numikrmsmp[4][-1][1] * np.pi / 180)
    # goal_config_rgt = robot_instance.get_gl_tcp("rgt_arm")



    # robot_instance.gen_meshmodel(rgba=(0,1,1,1),dual = "dual").attach_to(base)

    rrtc_planner = rrtc.RRTConnect(robot_instance)
    path_0 = rrtc_planner.plan(start_conf=robot_instance.get_jnt_values("rgt_arm"),
                             goal_conf=numikrmsmp[4][0][1]*np.pi/180,
                             obstacle_list=[object],
                             ext_dist=.1,
                             max_time=300,
                             component_name=component_name)
    path_1 = rrtc_planner.plan(start_conf=numikrmsmp[4][0][1]*np.pi/180,
                             goal_conf=numikrmsmp[4][-1][1] * np.pi / 180,
                             obstacle_list=[object],
                             ext_dist=.1,
                             max_time=300,
                             component_name=component_name)

    # with open("test_newplanner.pickle", "wb") as file:
    #     pickle.dump( path, file)
    # for item in numikrmsmp[4]:
    #     robot_instance.fk("rgt_arm", item[1]*np.pi/180)
    #     robot_instance.gen_meshmodel().attach_to(base)
    # base.run()



    # print(keyposes)
    # base.run()
    # start_hnd_pos = np.array([0.8, -0.5, 1.3])
    # start_hnd_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi / 2)
    # goal_hnd_pos = np.array([0.8, -0.5, 1.25])
    # goal_hnd_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi / 2)
    # start_jntsangle = robot_instance.ik(component_name, start_hnd_pos, start_hnd_rotmat)
    # goal_jntsangle = robot_instance.ik(component_name, goal_hnd_pos, goal_hnd_rotmat)
    # robot_instance.fk(component_name, start_jntsangle)
    # gm.gen_frame(pos=start_hnd_pos, rotmat=start_hnd_rotmat).attach_to(base)
    # gm.gen_frame(pos=goal_hnd_pos, rotmat=goal_hnd_rotmat).attach_to(base)
    # robot_meshmodel = robot_instance.gen_meshmodel(tcp_loc_pos=start_hnd_pos, tcp_loc_rotmat=start_hnd_rotmat)

    # robot_meshmodel.attach_to(base)
    path = path_1
    jawwidth_list = []
    objpose_list = []
    for pose in path:
        # print(pose)
        robot_instance.fk(component_name, pose)
        # robot_meshmodel = robot_instance.gen_meshmodel()
        # robot_instance.gen_meshmodel().attach_to(base)
        jawwidth_list.append(0)
        objpose_list.append(np.eye(4))
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
        # robot.jaw_to(hand_name, jawwidth_path[counter[0]])
        robot_meshmodel = robot.gen_meshmodel()
        robot_meshmodel.attach_to(base)
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
    # jawwidth_list = [a[0]/1000 for a in jawwidthmp[4]]
    conf_list = path
    # objpose_list = objmsmp[4]
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
    # def update(textNode, task):
    #
    #     if textNode[0] is not None:
    #         textNode[0].detachNode()
    #         textNode[1].detachNode()
    #         textNode[2].detachNode()
    #     cam_pos = base.cam.getPos()
    #     textNode[0] = OnscreenText(
    #         text=str(cam_pos[0])[0:5],
    #         fg=(1, 0, 0, 1),
    #         pos=(1.0, 0.8),
    #         align=TextNode.ALeft)
    #     textNode[1] = OnscreenText(
    #         text=str(cam_pos[1])[0:5],
    #         fg=(0, 1, 0, 1),
    #         pos=(1.3, 0.8),
    #         align=TextNode.ALeft)
    #     textNode[2] = OnscreenText(
    #         text=str(cam_pos[2])[0:5],
    #         fg=(0, 0, 1, 1),
    #         pos=(1.6, 0.8),
    #         align=TextNode.ALeft)
    #     return task.again
    #
    #
    # cam_view_text = OnscreenText(
    #     text="Camera View: ",
    #     fg=(0, 0, 0, 1),
    #     pos=(1.15, 0.9),
    #     align=TextNode.ALeft)
    # testNode = [None, None, None]
    # taskMgr.add(update, "addobject", extraArgs=[testNode], appendTask=True)
    base.run()