import math
import numpy as np
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import motion.probabilistic.rrt_connect as rrtc
import basis.robot_math as rm

base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
# object
object = cm.CollisionModel("objects/Coffee_cup.stl")
object.set_pos(np.array([0.28, -0.5, 1.18]))
object.set_rgba([.5, .7, .3, 1])
object.attach_to(base)
# robot_s
component_name = 'rgt_arm'
robot_s = ur3d.UR3Dual()

# possible right goal np.array([0, -math.pi/4, 0, math.pi/2, math.pi/2, math.pi / 6])
# possible left goal np.array([0, -math.pi / 2, -math.pi/3, -math.pi / 2, math.pi / 6, math.pi / 6])

#スタート状態の定義
start_pos = np.array([0.1, -0.55, 1.2])
start_rotmat = rm.rotmat_from_euler(ai=math.pi/2, aj=math.pi, ak=0, axes='szxz')
start_conf = robot_s.ik(component_name=component_name, tgt_pos=start_pos, tgt_rot=start_rotmat)
#ゴール状態の定義
goal_pos = np.array([0.5, -0.5, 1.2])
goal_rotmat = rm.rotmat_from_euler(ai=math.pi/2, aj=math.pi, ak=0, axes='szxz')
goal_conf = robot_s.ik(component_name=component_name, tgt_pos=goal_pos, tgt_rot=goal_rotmat)

rrtc_planner = rrtc.RRTConnect(robot_s)
path = rrtc_planner.plan(component_name=component_name,
                         start_conf=start_conf,
                         goal_conf=goal_conf,
                         obstacle_list=[object],
                         ext_dist=.2,
                         rand_rate=70,
                         max_time=300)
# print(path)
for pose in path:
    # print(pose)
    robot_s.fk(component_name, pose)
    robot_meshmodel = robot_s.gen_meshmodel()
    robot_meshmodel.attach_to(base)
    robot_s.gen_stickmodel().attach_to(base)

base.run()
