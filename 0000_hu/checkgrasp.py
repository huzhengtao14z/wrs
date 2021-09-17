import math
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.yumi_gripper.yumi_gripper as yg
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as hnde
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode

base = wd.World(cam_pos=[0.1, 1, 1],w=960,
                 h=540, lookat_pos=[0, 0, 0])
gm.gen_frame().attach_to(base)

objname = "tubebig"
grippername = "hnde"

object_tube = cm.CollisionModel(f"objects/{objname}.stl")
object_tube.set_rgba([.9, .75, .35, 1])
object_tube.attach_to(base)

# gripper_s = yg.YumiGripper()
gripper_s = hnde.RobotiqHE()

grasp_info_list = gpa.load_pickle_file('tubebig', '.', f'graspdata/{grippername}_{objname}.pickle')

for grasp_info in grasp_info_list:
    jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
    gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    gripper_s.gen_meshmodel().attach_to(base)
    break

# for grasp_info in grasp_info_list:
#     jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
#     gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
#     gripper_s.gen_meshmodel(rgba=(1,0,0,0.01)).attach_to(base)

def update(textNode, task):
    if textNode[0] is not None:
        textNode[0].detachNode()
        textNode[1].detachNode()
        textNode[2].detachNode()
    cam_pos = base.cam.getPos()
    textNode[0] = OnscreenText(
        text = str(cam_pos[0])[0:5],
        fg = (1,0,0,1),
        pos = (1.0,0.8),
        align = TextNode.ALeft)
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