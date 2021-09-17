import math
import visualization.panda.world as wd
import modeling.geometric_model as gm
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
if __name__ == '__main__':
    base = wd.World(cam_pos=[2, 1, 3], w=960,
                    h=540, lookat_pos=[0, 0, 1.1])
    gm.gen_frame().attach_to(base)

    object = cm.CollisionModel("./objects/bunnysim.stl")
    object.set_pos(np.array([1.05, -.3, 1.3]))
    object.set_rgba([.5, .7, .3, 1])
    object.attach_to(base)
    component_name = 'rgt_arm'
    # robot_instance = ur3d.UR3Dual()
    robot_instance = ur3ed.UR3EDual()
    start_hnd_pos = np.array([0.9, -0.5, 1.3])
    start_hnd_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi / 2)
    goal_hnd_pos = np.array([0.4, -0.3, 1.3])
    goal_hnd_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi / 2)
    start_jntsangle = robot_instance.ik(component_name,start_hnd_pos,start_hnd_rotmat)
    robot_instance.fk(component_name, start_jntsangle)
    # gm.gen_frame(pos=start_hnd_pos, rotmat=start_hnd_rotmat).attach_to(base)
    # gm.gen_frame(pos=goal_hnd_pos, rotmat=goal_hnd_rotmat).attach_to(base)
    robot_meshmodel = robot_instance.gen_meshmodel(tcp_loc_pos=start_hnd_pos, tcp_loc_rotmat=start_hnd_rotmat)

    robot_meshmodel.attach_to(base)
    base.run()