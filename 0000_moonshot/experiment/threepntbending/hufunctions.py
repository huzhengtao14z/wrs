import config
import localenv.envloader as el
import graspplanner.graspmap_utils as gu
import motionplanner.motion_planner as m_planner
import utils.run_utils as ru
import utils.drawpath_utils as du
import utils.pcd_utils as pcdu
import utils.phoxi_locator as pl
import utiltools.robotmath as rm
import utils.phoxi as phoxi
import pickle
from utiltools.robotmath import rodrigues
import numpy as np
# from robotcon.ur3edual import Ur3EDualUrx
#
# urx = Ur3EDualUrx(rbt)



def get_currnetPosRot(rbt,rbtx, rotmat = None):
    rbt = rbt
    rbtx = rbtx
    rbt.movearmfk(rbtx.getjnts("lft"), armname="lft")
    tcp = rbt.gettcp("lft")
    print(f"The tcp is {tcp}")

    if rotmat:
        gripperrotmat44 = rotmat
        gripperrotmat44 = np.array([[-1.66533454e-16, 9.96194698e-01, -8.71557481e-02,
                           6.56089380e+01],
                          [9.96194698e-01, -7.59612442e-03, -8.68240941e-02,
                           1.33866356e+01],
                          [-8.71557481e-02, -8.68240941e-02, -9.92403876e-01,
                           1.50665656e+02],
                          [0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                           1.00000000e+00]])
        objorigion = np.dot(rm.homobuild(*tcp), gripperrotmat44)
        print(objorigion)
        objorigion_pos = objorigion[:3,3]
        objorigion_rot = objorigion[:3,:3]
        print(f"The obj origin is : pos{objorigion_pos}, rot{objorigion_rot}")
        return objorigion

    return tcp

def alignTo(rbt = None, rbtx = None, armname = "lft"):
    if rbtx is None:
        print("Connect to the robot first")
        return
    print("align to")
    armname = armname
    jnts = rbtx.getjnts(armname)
    rbt.movearmfk(jnts, armname=armname)
    eepos, eerot = rbt.getee(armname=armname)

    if np.dot(eerot[:3, 2], np.array([0, 0, 1])) > 0:
        rottt = np.eye(3)
    else:
        rottt = rodrigues([1, 0, 0], 180)

    newjnts = rbt.numikmsc(eepos, rottt, seedjntagls = jnts,armname=armname)
    print(f"old jnt is : {jnts}")
    print(f"new jnt is : {newjnts}")
    rbtx.movejntssgl(newjnts, armname=armname)
    # self.set2RealPos()


if __name__ == "__main__":
#Test
    base, env = el.loadEnv_wrs()
        # slopecm = el.loadvslope(env)
    obscmlist = env.getstationaryobslist() + env.getchangableobslist()

    # load robot
    rbt, rbtmg, rbtball = el.loadUr3e()
    rbtx = el.loadUr3ex(rbt)

    get_currnetPosRot(rbt, rbtx)
    alignTo(rbt, rbtx)