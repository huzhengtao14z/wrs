import motionplanner.motion_planner as m_planner
import motionplanner.rbtx_motion_planner as m_planner_x
# import motionplanner.rbtx_motion_planner as m_planner_x
import hufunctions
import utils.phoxi as phoxi
import utils.phoxi_locator as pl
from utils.run_script_utils import *
import localenv.envloader as el

if __name__ == '__main__':
    '''
    set up env and param
    '''
    base, env = el.loadEnv_wrs()
    rbt, rbtmg, rbtball = el.loadUr3e()
    # rbt.opengripper(armname="rgt")
    # rbt.opengripper(armname="lft")
    rbtx = el.loadUr3ex(rbt)
    '''
    init class
    '''
    mp_lft = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="lft")
    mp_lftx = m_planner_x.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="lft")
    mp_rgt = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="rgt")
    mp_rgtx = m_planner_x.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="rgt")
    # mp_lftx.goto_init_x()
    # mp_rgtx.goto_init_x()

    lft_jnts_current = rbtx.getjnts(armname="lft")
    rbt.movearmfk(lft_jnts_current, armname = "lft")
    lft_ee_current = rbt.getee(armname = "lft")
    lft_tcp_current = rbt.gettcp(armname = "lft")

    # lft_jnts_current = mp_lftx.get_armjnts()
    # ee_jnts_current = mp_lftx.get_ee()
    # tcp_jnts_current = mp_lftx.get_tcp()

    print("-----------------")
    print(lft_jnts_current)
    print("-----------------")
    print(lft_ee_current)
    print("-----------------")
    print(lft_tcp_current)
    print("-----------------")
    base.run()
