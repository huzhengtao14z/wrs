import copy

import config
import localenv.envloader as el
import graspplanner.grasp_planner as gp
import motionplanner.motion_planner as m_planner
import utils.run_utils as ru
import utils.drawpath_utils as du
import utils.pcd_utils as pcdu
import utils.phoxi_locator as pl
import utiltools.robotmath as rm
import utils.phoxi as phoxi
import pickle
import motionplanner.rbtx_motion_planner as m_planner_x
from utiltools.robotmath import rodrigues
import numpy as np
from trimesh.primitives import Box
from trimesh.primitives import Cylinder
import environment.collisionmodel as cm

class Phoxihelper():
    def __init__(self, phoxi_f_path = "hzt.pkl"):

        self.phoxi_f_path = phoxi_f_path
        self.phxi = phoxi.Phoxi(host=config.PHOXI_HOST)
        self.phxilocator = pl.PhxiLocator(phoxi, amat_f_name=config.AMAT_F_NAME_NPY)
        self.set_obj()
        self.set_recog_range()


    def set_obj(self, name = "bigshaft"):
        self.name = name

    def get_obj(self):
        return self.name

    def set_recog_range(self, x_range = (400, 900), y_range= (0, 200), z_range= (780+135, 1000)):
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range

    def get_objcm(self):
        return self.objcm

    def recognize(self, match_rotz = True, newphoto = True, save = True):
        match_rotz = match_rotz
        newphoto = newphoto
        save = save

        pcd = None

        if newphoto:
            grayimg, depthnparray_float32, pcd = self.phxi.dumpalldata(f_name="img/" + self.phoxi_f_path)
            if save:
                with open("/home/hlabbunri/Desktop/hu/wrs-2020/0000_wrs/data/" + self.name + "pkl", "wb") as file:
                    pickle.dump([grayimg, depthnparray_float32, pcd], file)
        else:
            try:
                with open("/home/hlabbunri/Desktop/hu/wrs-2020/0000_wrs/data/" + self.name + "pkl", "rb") as file:
                    tempdata = pickle.load(file)
                grayimg, depthnparray_float32, pcd = tempdata
            except:
                print("please set newphoto as True")

        # grayimg, depthnparray_float32, pcd = phxi.dumpalldata(f_name="img/" + phoxi_f_path)
        paintingobj = \
            ru.get_obj_from_phoxiinfo_withmodel_nobgf(self.phxilocator, self.name + ".stl",
                                                      phoxi_f_name=self.phoxi_f_path, match_rotz=match_rotz, load=True,
                                                      x_range=self.x_range, y_range=self.y_range, z_range=self.z_range)

        self.amat = np.load(config.ROOT + "/camcalib/data/" + config.AMAT_F_NAME_NPY)
        if pcd is not None:
            pointdata = pcdu.trans_pcd(pcd, self.amat)
            objmat = paintingobj.objmat4
            pcdu.show_pcd(pointdata, rgba=(1, 1, 1, .1))
            paintingobj.show_objpcd(rgba=(1, 0, 0, 1))
            paintingobj.show_objcm(rgba=(1, 1, 0, 0.3))

            pickle.dump(paintingobj.pcd, open(f"pcd_{self.name}.pkl", "wb"))
            self.objmat = objmat
            self.objcm = paintingobj.objcm
            base.pggen.plotAxis(base.render, self.objcm.gethomomat()[:3, 3], self.objcm.gethomomat()[:3, :3])
            return self.objmat

        else:
            return 0

if __name__ == '__main__':
    import manipulation.grip.robotiqhe.robotiqhe as rtqhe
    import config
    import pickle

    base, env = el.loadEnv_wrs()
    rbt, rbtmg, rbtball = el.loadUr3e()
    hndfa = rtqhe.HandFactory()
    rbtx = el.loadUr3ex(rbt)
    motion_planner_lft = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="lft")
    motion_planner_x_lft = m_planner_x.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="lft")
    motion_planner_x_lft.goto_init_x()
    rbtx.opengripper(armname="lft")
    base.run()
    # newrecognition = False

    ###### Load Phoxi
    # phoxihelper = Phoxihelper()
    # objmat_init = phoxihelper.recognize(newphoto=True, save=False)
    # objcm = phoxihelper.get_objcm()
    ######

    objholder = cm.CollisionModel(objinit=Box(box_extents = [100,100,100]))
    objholder.setColor(.32, .32, .3, 1)
    objholder.setpos([719.56843365+3.2686120652925292,  35.7432725 -1.8871340557731007, 780+50])
    objholder.reparentTo(base.render)
    motion_planner_lft.add_obs(objholder)
    # env.addchangableobs(base.render, objcm, [16*30, 0, 785], np.eye(3))
    # [array([719.56843365, 35.7432725, 892.12734689]), array([[-0.01111138, 0.99899212, 0.04348881],
    #                                                          [0.99985138, 0.01052659, 0.013653],
    #                                                          [0.01318145, 0.04363405, -0.99896062]])]
    obj_pos = np.array([719.56843365+2.57+5.5-3.15676327-0.27899846 ,  35.7432725 -0.69-5.5- 4.21989917 +5.29717557, 891.12734689-6-28+4.22+5.87+1.85926127- 4.49691663])
    before = rm.homobuild(np.array([673.19317469, 60.81176012, 902.08560022]),
                          np.array([[0.15771317, 0.66228295, 0.73246696],
                                    [0.86765792, 0.26121558, -0.42300845],
                                    [-0.47148307, 0.70224476, -0.53343791]]))

    after = rm.homobuild(np.array([673.00725898, 59.52574216, 902.74192666]),
                         np.array([[0.14603211, 0.6622587, 0.73490682],
                                   [0.87430874, 0.26118909, -0.40910206],
                                   [-0.46288104, 0.70227749, -0.54087657]]))

    offset_b2a = np.dot(after, np.linalg.inv(before))
    print(offset_b2a)


    obj_rot = np.dot( rm.rodrigues([0, 0, 1], 150), np.dot(rm.rodrigues([0,1,0], -90+57.74), rm.rodrigues([1,0,0], 31)))




    objmat_init = np.dot(offset_b2a, rm.homobuild(obj_pos, obj_rot))
    objcm = cm.CollisionModel(config.ROOT + "/obstacles/bigshaft.stl")
    objcm.setColor(.12, .62, .3, 1)
    objcm.sethomomat(objmat_init)
    objcm.reparentTo(base.render)

    objcm_goal = copy.deepcopy(objcm)
    objmat_goal = rm.homobuild([1000, 150, 780+150], rot= rm.rodrigues((0, 1, 0), -90))
    base.pggen.plotAxis(base.render, srot=objmat_goal[:3,:3], spos=objmat_goal[:3,3])
    objcm_goal.sethomomat(objmat_goal)
    objcm_goal.reparentTo(base.render)

    graspplanner = gp.GraspPlanner(hndfa)
    grasps = graspplanner.load_pregrasp("bigshaft")

    def show_all_grasp(objconfigname = "init", rgba = (1,1,0,0.03)):

        if objconfigname == "init":
            for grasp in grasps:
                gripper = hndfa.genHand(grasp[0])
                handmat = np.dot(objmat_init, grasp[2])
                gripper.sethomomat(handmat)
                gripper.setColor(rgba)
                gripper.reparentTo(base.render)
        elif objconfigname == "goal":
            for grasp in grasps:
                gripper = hndfa.genHand(grasp[0])
                handmat = np.dot(objmat_goal, grasp[2])
                gripper.sethomomat(handmat)
                gripper.setColor(rgba)
                gripper.reparentTo(base.render)

    show_all_grasp("init")


    ee = rbt.getee("lft")
    tcp = rbt.gettcp("lft")
    # print("ee", ee)
    # print("tcp", tcp)
    tcpToee = np.dot(np.linalg.inv(rm.homobuild(ee[0], ee[1])), rm.homobuild(tcp[0], tcp[1]))
    # tcpToee = np.dot(rm.homobuild(ee[0], ee[1]), np.linalg.inv(rm.homobuild(tcp[0], tcp[1])))
    # print("tcpToee", tcpToee)

    debug_mat = rm.homobuild([0, 100, 0], np.eye(3))
    # objmat_goal = np.dot(debug_mat, objmat_init)

    # objmat_goal = rm.homobuild([800, 400, 900], rot=np.dot(rm.rodrigues((0, 0, 1), 170), rm.rodrigues((0, 1, 0), 90)))


    show_all_grasp("goal")
    objmat_pair = [objmat_init, objmat_goal]


    # with open(config.ROOT + "path_data.pkl", 'rb') as file:
    #     path = pickle.load(file)
    # jawwidth = path[1]
    #
    # path = path[0]
    # motion_planner_lft.ah.show_animation_pick(path, objcm, objrelpos, objrelrot, jawwidth)

    for i,grasp in enumerate(grasps):
        if i == 39:
            continue
        objrelpos, objrelrot = motion_planner_lft.get_rel_posrot(grasp, objmat_init[:3, 3], objmat_init[:3, :3])

        path= motion_planner_lft.plan_initnpicknplace(grasp, objmat_pair, objcm, objrelpos, objrelrot, use_msc=True, use_pickupprim=True,
                        use_placedownprim=True, start=None, pickupprim_len=50, placedownprim_len=50)

        if path is not None:
            # print(path)
            jawwidth = path[1]
            path = path[0]
            print("grasp ID:", i)
            motion_planner_lft.ah.show_animation_pick(path, objcm, objrelpos, objrelrot, jawwidth)
            motion_planner_x_lft.movemultipaths(path)


            # pickle.dump(path,
            #             open(config.ROOT + "path_data.pkl",
            #                  'wb'))
            base.run()
            break
        ######


        # gripper = hndfa.genHand(grasp[0])
        # handmat = np.dot(objmat_init, grasp[2])
        # gripper.sethomomat(handmat)
        # gripper.reparentTo(base.render)
        # handmat = np.dot(handmat, np.linalg.inv(tcpToee))
        #
        # eerot = handmat[:3, :3]
        # eepos = handmat[:3, 3]
        # base.pggen.plotAxis(base.render,srot = eerot, spos = eepos)
        #
        # print("handmat", handmat)
        #
        # ik = rbt.numik(eepos=eepos, eerot = eerot, armname="lft")
        # if ik is not None:
        #     print("no IK")
        #     rbt.movearmfk(ik, armname="lft")
        #     ur3mnp = rbtmg.genmnp(rbt, togglejntscoord=False, toggleendcoord=True)
        #     ur3mnp.reparentTo(base.render)


    # print("objmat", objmat_init)

    base.run()
    # motion_planner_lft.add_obs(objcm)
    # if newrecognition:
    #     objmat_init = phoxihelper.recognize(newphoto = False, save = False)
    # else:
    # objmat_init =  np.array([[-4.15629808e-02,  8.24298233e-01,  5.64628145e-01,  7.90742641e+02],
    #                 [-9.98984500e-01, -4.41218211e-02, -9.12323342e-03,  1.43745286e+02],
    #                 [1.73921568e-02,  -5.64433954e-01,  8.25294993e-01,  8.32381481e+02],
    #                 [0.00000000e+00,   0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

