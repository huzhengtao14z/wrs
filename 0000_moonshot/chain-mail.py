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
import robot_sim._kinematics.collision_checker as cc
import robot_sim.end_effectors.grippers.gripper_interface as gi
# import robot_sim.
if __name__ == '__main__':
    base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960,
                    h=540, lookat_pos=[0, 0, 0.0])
    gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)
    robot_instance = ur3ed.UR3EDual()
    # object = cm.CollisionModel("./objects/bunnysim.stl")
    # object.set_pos(np.array([1.05, -.3, 1.3]))
    # object.set_rgba([.5, .7, .3, 1])
    # object.attach_to(base)
    # cm.gen_box().attach_to(base)
    # box = gm.gen_box(extent=[0.01,0.01,0.06],homomat=np.eye(4),rgba=[1,1,0,1])
    start = np.array([0.010, 0.01, .010])
    end = np.array([0.00, 0.0, 0.01])
    def cylinder_link_start_end(start, end):
        start = start
        end = end
        height = np.linalg.norm(end - start, ord = 2, axis = 0, keepdims=True)[0]
        vector = rm.unit_vector(end - start)
        rot3 = rm.rotmat_between_vectors(np.array([0,0,1]),vector)
        rot4 = rm.homomat_from_posrot(pos = start, rot = rot3)
        cylinder = cm.gen_cylinder(radius=0.001, height=height, section=30, homomat=rot4)
        return cylinder

    def capsule_link_start_end(start, end):
        start = start
        end = end
        thickness = 0.001
        cylinder = cm.gen_capsule(spos=start, epos=end, radius = 0.0005, section=[5, 5])
        return cylinder

    cylinder1 = capsule_link_start_end(start,end)
    cylinder1.attach_to(base)
    start = np.array([0.020, 0.02, .010])
    end = np.array([0.00, 0.020, 0.02])
    cylinder2 = capsule_link_start_end(start, end)
    cylinder2.attach_to(base)

    # cylinder1 = cm.CollisionModel(cylinder1)
    # cylinder2 = cm.CollisionModel(cylinder2)
    # checker = gi.GripperInterface()
    # answer = checker.is_mesh_collided([cylinder2,cylinder1])
    # collision_checker = cc.CollisionChecker()

    answer = cylinder1.is_mcdwith(objcm_list=[cylinder2])
    print(answer)
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