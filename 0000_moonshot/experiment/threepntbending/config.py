import os

'''
param
'''
ID_DICT = {
    "0524": [13, 52],
    "0525": [2, 401],
    "cube": [33, 56],  # real_exp_cube [30, 31][33, 56]
    "cylinder_cad": [54, 22],
    "cylinder_pcd": [3, 10],
    "helmet": [22, 654],
    "bunny": [42, 44],
    "temp": [34, 293],
    "cylinder_mtp": [55, 290],
}

PHOXI_HOST = "10.0.1.31:18300"
ROOT = os.path.abspath(os.path.dirname(__file__))
# AMAT_F_NAME = "phoxi_calibmat_0615.pkl"
# AMAT_F_NAME = "phoxi_calibmat_1222.pkl"
AMAT_F_NAME_NPY = "phoxi_calibmat_220324.npy"
PEN_STL_F_NAME = "pentip_short.stl"
# PEN_STL_F_NAME = "pentip.stl"
IPURX = '10.0.2.11'

PREGRASP_REL_PATH = "/graspplanner/pregrasp/"
GRASPMAP_REL_PATH = "/graspplanner/graspmap/"
# GRASPMAP_REL_PATH = "/graspplanner/graspmap/temp/"
MOTIONSCRIPT_REL_PATH = "/motionscript/"
PENPOSE_REL_PATH = "/log/penpose/"

'''
robot
'''
STATE = {}
state_template = {
    "objname": None,
    "obstaclecmlist": None,
    "assembly_pos": None,
    "startmat4": None,
    "goalmat4": None,
    "planned": None,
    "objmsmp": None,
    "numikrmsmp": None,
    "jawwidthmp": None,
    "planner": None,
}