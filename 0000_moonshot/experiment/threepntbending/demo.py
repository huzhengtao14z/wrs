import motionplanner.motion_planner as m_planner
import utils.phoxi as phoxi
import utils.phoxi_locator as pl
from utils.run_script_utils import *


def load_path(folder_name, method_name, grasp_id):
    for dirpath, dirnames, filenames in os.walk(f"{folder_name}/"):
        for f in filenames:
            if f.endswith(".pkl") and f.find(f"{method_name}_{grasp_id}_") != -1:
                print(f)
                return pickle.load(open(os.path.join(folder_name, f), "rb"))
    print("File not found!")
    return None


if __name__ == '__main__':
    '''
    set up env and param
    '''
    base, env = el.loadEnv_wrs()
    rbt, rbtmg, rbtball = el.loadUr3e()
    # rbtx = el.loadUr3ex(rbt)
    rbt.opengripper(armname="rgt")
    rbt.opengripper(armname="lft")

    pen = el.loadObj("pentip.stl")

    '''
    init class
    '''
    motion_planner_rgt = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="rgt")
    mp_lft = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="lft")
    # mp_x_rgt = m_plannerx.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="rgt")
    # mp_x_lft = m_plannerx.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="lft")
    phxilocator = pl.PhxiLocator(phoxi, amat_f_name=config.AMAT_F_NAME)

    objmat4_list = pickle.load(open(config.ROOT + config.PENPOSE_REL_PATH + "/bucket_cad_circle.pkl", "rb"))
    grasp_list = pickle.load(
        open(config.ROOT + config.PREGRASP_REL_PATH + config.PEN_STL_F_NAME.split(".stl")[0] + "_pregrasps.pkl", "rb"))
    folder_name = config.ROOT + "/log/path/nlopt"
    method_name = "boxcolw45"
    grasp_id = 0

    mp_lft.ah.show_objmat4_list_pos(objmat4_list, rgba=(1, 0, 0, 1))
    try:
        objrelpos, objrelrot, path_draw = load_path(folder_name, method_name, grasp_id)
        path_end = mp_lft.get_moveup_path(path_draw[-1], pen, objrelpos, objrelrot)
        print(path_end)
        mp_lft.ah.show_animation_hold(path_draw + path_end, pen, objrelpos, objrelrot)
    except:
        path_draw = load_path(folder_name, method_name, grasp_id)
        mp_lft.ah.show_animation(path_draw)
    base.run()

    # mp_x_rgt.goto_init_x()
    # rbtx.opengripper(armname="lft")
    # mp_x_lft.goto_init_x()
    #
    # mp_x_lft.goto_armjnts_x(path_draw[0])
