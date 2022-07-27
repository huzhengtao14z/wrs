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
    base, env = el.loadEnv_wrs()
    rbt, rbtmg, rbtball = el.loadUr3e()
    hndfa = rtqhe.HandFactory()
    rbtx = el.loadUr3ex(rbt)
    motion_planner_lft = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="lft")
    motion_planner_x_lft = m_planner_x.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="lft")
    # motion_planner_rgt = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="rgt")
    # motion_planner_x_rgt = m_plannerx.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="rgt")
    motion_planner_x_lft.goto_init_x()
    rbtx.opengripper(armname="lft")
    # rbtx.closegripperwithimpedance(armname="rgt")
    # newrecognition = False
    phoxihelper = Phoxihelper()
    objmat_init = phoxihelper.recognize(newphoto=True, save=False)
    print("objmat", objmat_init)
    base.run()
    objcm = phoxihelper.get_objcm()
    objholder = cm.CollisionModel(objinit=Cylinder(radius=40, height=10))
    objholder.setColor(.32, .32, .3, 1)
    objholder.reparentTo(base.render)
    env.addchangableobs(base.render, objcm, [19.5*30, 20, 780], np.eye(3))
    # motion_planner_lft.add_obs(objcm)
    # if newrecognition:
    #     objmat_init = phoxihelper.recognize(newphoto = False, save = False)
    # else:
    # objmat_init =  np.array([[-4.15629808e-02,  8.24298233e-01,  5.64628145e-01,  7.90742641e+02],
    #                 [-9.98984500e-01, -4.41218211e-02, -9.12323342e-03,  1.43745286e+02],
    #                 [1.73921568e-02,  -5.64433954e-01,  8.25294993e-01,  8.32381481e+02],
    #                 [0.00000000e+00,   0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
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

    objmat_goal = rm.homobuild([1000, 250, 780+150+80], rot=np.dot(rm.rodrigues((0, 0, 1), 180), rm.rodrigues((0, 1, 0), 90)))
    base.pggen.plotAxis(base.render, srot=objmat_goal[:3,:3], spos=objmat_goal[:3,3])
    objcm.sethomomat(objmat_goal)
    objcm.reparentTo(base.render)
    show_all_grasp("goal")
    objmat_pair = [objmat_init, objmat_goal]

    for grasp in grasps:
        objrelpos, objrelrot = motion_planner_lft.get_rel_posrot(grasp, objmat_init[:3, 3], objmat_init[:3, :3])

        ######
        # path_init2pick = motion_planner_lft.plan_gotopick(grasp, objmat_init, objcm, objrelpos, objrelrot)
        # if path_init2pick is not None:
        #     path_init2pick_jawwith = [50 for i in path_init2pick]
        #     path_pick2init = path_init2pick[::-1]
        #     path_init2pick.extend(path_pick2init)
        #     gripper = hndfa.genHand(grasp[0])
        #     handmat = np.dot(objmat_init, grasp[2])
        #     gripper.sethomomat(handmat)
        #     gripper.setColor((0,1,0,1))
        #     gripper.reparentTo(base.render)
        #     motion_planner_lft.ah.show_animation_hold(path_init2pick, objcm, objrelpos, objrelrot)
        #     base.run()
        #     motion_planner_x_lft.movepath(path_init2pick)
        #     break
        ######

        ######
        path= motion_planner_lft.plan_initnpicknplace(grasp, objmat_pair, objcm, objrelpos, objrelrot, use_msc=True, use_pickupprim=True,
                        use_placedownprim=True, start=None, pickupprim_len=50, placedownprim_len=70)

        if path is not None:
            # print(path)
            jawwidth = path[1]

            path = path[0]

            print(jawwidth)
            # gripper = hndfa.genHand(grasp[0])
            # handmat = np.dot(objmat_init, grasp[2])
            # gripper.sethomomat(handmat)
            # gripper.setColor((0,1,0,1))
            # gripper.reparentTo(base.render)
            # gripper = hndfa.genHand(grasp[0])
            # gripper.sethomomat(np.dot(objmat_goal, grasp[2]))
            # gripper.setColor((0, 1, 0, 1))
            # gripper.reparentTo(base.render)
            # motion_planner_lft.ah.show_animation_hold(path, objcm, objrelpos, objrelrot)
            motion_planner_lft.ah.show_animation_pick(path, objcm, objrelpos, objrelrot, jawwidth)
            # base.run()
            motion_planner_x_lft.movemultipaths(path)
            import pickle

            pickle.dump(path,
                        open(config.ROOT + "path_data.pkl",
                             'wb'))
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


    print("objmat", objmat_init)

    base.run()