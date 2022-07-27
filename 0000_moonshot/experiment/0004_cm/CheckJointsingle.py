#-*-coding:utf-8-*-
import pickle
import time
import numpy as np
import os
import cv2
from panda3d.core import *
import utiltools.robotmath as rm
import cv2 as cv
import cv2.aruco as aruco
# from robotcon.rpc.hndcam.hndcam_client import HndCam

class HndCam(object):

    def __init__(self, rgtcamid = [1,2], lftcamid = [0,3]):
        self.rgtcamid = rgtcamid
        self.lftcamid = lftcamid
        # pose of the camera in the local coordinate system of right hand
        # this_dir, this_filename = os.path.split(__file__)
        this_dir = "."
        rgthndcamparr = os.path.join(this_dir, "hndcamparameters", "right_arm_camera_calib__.yaml")
        # print(rgthndcamparr)
        cv_file = cv2.FileStorage(rgthndcamparr, cv2.FILE_STORAGE_READ)
        self.rgtmtx = cv_file.getNode("camera_matrix").mat()
        self.rgtdist = cv_file.getNode("dist_coeff").mat()

        lfthndcamparr = os.path.join(this_dir, "hndcamparameters", "left_arm_camera_calib__.yaml")
        cv_file = cv2.FileStorage(lfthndcamparr, cv2.FILE_STORAGE_READ)
        self.lftmtx = cv_file.getNode("camera_matrix").mat()
        self.lftdist = cv_file.getNode("dist_coeff").mat()

        print(self.rgtmtx,self.rgtdist)
        self.T_CR_rgt = np.array([[-9.99442764e-01, -2.18179570e-02, 2.52614411e-02, -4.21361377e+00],
                                 [2.07905605e-02, -9.98973601e-01, -4.02428632e-02, -2.62700248e+01],
                                 [2.61135184e-02, -3.96952281e-02, 9.98870549e-01, -8.44215037e+01],
                                 [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        self.T_CL_lft = np.array([[9.98987167e-01, 3.75007165e-02, -2.48664049e-02, 1.09009752e+01],
                                  [-3.71338501e-02, 9.99196892e-01, 1.50548575e-02, 3.98462236e+01],
                                  [2.54110028e-02, -1.41162236e-02, 9.99577417e-01, -8.19180008e+01],
                                  [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])



    def getrc0img(self):
        camcap = cv2.VideoCapture(self.rgtcamid[0]+cv2.CAP_DSHOW)
        img = camcap.read()[1]
        camcap.release()

        return img

    def getrc1img(self):
        camcap = cv2.VideoCapture(1+cv2.CAP_DSHOW)
        img = camcap.read()[1]
        camcap.release()

        return img

    def getlc0img(self):
        camcap = cv2.VideoCapture(self.lftcamid[0] + cv2.CAP_DSHOW)
        # camcap = cv2.VideoCapture('test.mp4')
        fps = camcap.get(cv2.CAP_PROP_FPS)
        img = camcap.read()[1]
        camcap.release()
        print(fps, "fps")

        return img

    def getlc1img(self):
        camcap = cv2.VideoCapture(0 + cv2.CAP_DSHOW)
        img = camcap.read()[1]
        camcap.release()

        return img

    def getObjPoseRgtCam(self, eepos, eerotmat, marker_id=5):
        """
        get the homo pose of obj using rgtcam

        :param eepos: endeffector pose
        :param eerotmat:  end effector rotmat
        :return:

        author: weiwei, mmd
        date: 20180925
        """

        # hcc = cv.VideoCapture(0)
        frame = self.getrc0img()
        # connectToCam = rpyc.connect("10.2.0.60", 18300)
        # frame = pickle.loads(connectToCam.root.exposed_getrc0img())
        # # frame = cv2.flip(frame,-1)
        # fopen = open('photoNew.pickle', 'wb')
        # pickle.dump(frame, fopen)
        # fopen.close()

        rgtarmrot = eerotmat
        rgtarmpos = eepos
        T_RGTW_a = np.eye(4)
        T_RGTW_a[:3, :3] = rgtarmrot
        T_RGTW_a[:3, 3] = rgtarmpos
        T_CW_a = np.dot(T_RGTW_a, self.T_CR_rgt)
        det_T_MC = np.eye(4)



        # base.pggen.plotSphere(base.render, pos=T_CW_a[:3, 3], radius=20)
        # base.pggen.plotAxis(base.render, spos=T_CW_a[:3, 3], pandamat4=base.pg.np4ToMat4(T_CW_a))


        T_MW_a = None
        for i in range(1):
            detected = False
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if np.all(ids != None):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.035, self.rgtmtx,
                                                                self.rgtdist)  # the second value is the marker size in meters
                for i in range(ids.shape[0]):
                    if ids[i] == marker_id:
                        det_rvec = rvec[i]  # check this if ERROR
                        print( "------------------------det_rvec------\n",det_rvec)
                        det_tvec = 1000 * tvec[i]  # check this if ERROR

                        det_rmat, _ = cv2.Rodrigues(det_rvec)
                        det_T_MC[:3, :3] = det_rmat
                        det_T_MC[:3, 3] = det_tvec[0]
                        T_MW_a = np.dot(T_CW_a, det_T_MC)  # ought to be the left hand pose
                        detected = True
                        break
                if detected:
                    break
        final_T = T_MW_a

        # base.pggen.plotAxis(base.render, spos=final_T[:3, 3], pandamat4=base.pg.np4ToMat4(final_T))

        return final_T


    def getObjPoseLftCam(self, eepos, eerotmat, marker_id=1):
        """
        get the homo pose of obj using rgtcam

        :param eepos: endeffector pose
        :param eerotmat:  end effector rotmat
        :return:

        author: weiwei, mmd
        date: 20180925
        """
        # hcc = HndCamx(host="10.2.0.60:18300")
        frame = self.getlc0img()
        # frame = np.flip(frame)
        # cv2.imshow("12",frame)
        # cv2.waitKey(0)
        # frame = np.flip(frame)


        lftarmrot = eerotmat
        lftarmpos = eepos
        T_RGTW_a = np.eye(4)
        T_RGTW_a[:3, :3] = lftarmrot
        T_RGTW_a[:3, 3] = lftarmpos
        T_CW_a = np.dot(T_RGTW_a, self.T_CL_lft)
        det_T_MC = np.eye(4)
        # base.pggen.plotAxis(base.render, spos=T_CW_a[:3, 3], pandamat3=base.pg.npToMat3(T_CW_a[:3, :3]))
        T_MW_a = None

        for i in range(1):
            detected = False
            # ret, frame = self.rgtcap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if np.all(ids != None):

                ct = np.sum(corners[0], axis=1) / corners[0].shape[1]
                ct.astype(int)
                ct=ct[0]
                ct=tuple(ct)
                cv.circle(img=frame, center=ct, radius=100, color=(0,255,0), thickness=5, lineType=8, shift=0)
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.035, self.lftmtx,self.lftdist)  # the second value is the marker size in meters
                for i in range(ids.shape[0]):
                    if ids[i] == marker_id:
                        det_rvec = rvec[i]  # check this if ERROR
                        det_tvec = 1000 * tvec[i]  # check this if ERROR
                        det_rmat, _ = cv2.Rodrigues(det_rvec)
                        det_T_MC[:3, :3] = det_rmat
                        det_T_MC[:3, 3] = det_tvec[0]
                        print(det_tvec)
                        T_MW_a = np.dot(T_CW_a, det_T_MC)  # ought to be the left hand pose
                        detected = True
                        break
                if detected:
                    break
        final_T = T_MW_a
        print(final_T)
        return frame


    def getObjfromVideo(self, frame, eepos, eerotmat, marker_id=1):
        """
        get the homo pose of obj using rgtcam

        :param eepos: endeffector pose
        :param eerotmat:  end effector rotmat
        :return:

        author: weiwei, mmd
        date: 20180925
        """

        # hcc = cv.VideoCapture(0)
        # frame = self.getlc0img()
        frame = frame
        # connectToCam = rpyc.connect("10.2.0.60", 18300)
        # frame = pickle.loads(connectToCam.root.exposed_getrc0img())
        # # frame = cv2.flip(frame,-1)
        # fopen = open('photoNew.pickle', 'wb')
        # pickle.dump(frame, fopen)
        # fopen.close()

        rgtarmrot = eerotmat
        rgtarmpos = eepos
        T_RGTW_a = np.eye(4)
        T_RGTW_a[:3, :3] = rgtarmrot
        T_RGTW_a[:3, 3] = rgtarmpos
        T_CW_a = np.dot(T_RGTW_a, self.T_CR_rgt)
        det_T_MC = np.eye(4)



        # base.pggen.plotSphere(base.render, pos=T_CW_a[:3, 3], radius=20)
        # base.pggen.plotAxis(base.render, spos=T_CW_a[:3, 3], pandamat4=base.pg.np4ToMat4(T_CW_a))


        T_MW_a = None
        for i in range(1):
            detected = False
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if np.all(ids != None):
                pts = np.array(corners[0],dtype = np.int32)
                cv.polylines(img=frame,pts=pts, isClosed=True, color=(255, 0, 0), thickness=5, lineType=8, shift=0)

                ct = np.sum(corners[0], axis=1) / corners[0].shape[1]
                ct.astype(int)
                ct = tuple(ct[0])
                cv.circle(img=frame, center=ct, radius=100, color=(0, 255, 0), thickness=5, lineType=8, shift=0)

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.035, self.rgtmtx,
                                                                self.rgtdist)  # the second value is the marker size in meters
                # print("------------------------rvec------\n", rvec,type(rvec), "\n","------------------------rvec------\n", )
                # print("------------------------tvec------\n", tvec, type(tvec), "\n",
                #       "------------------------tvec------\n", )
                for i in range(ids.shape[0]):

                    cv.aruco.drawAxis(image=frame, cameraMatrix=self.rgtmtx, distCoeffs=self.rgtdist, rvec=rvec[i], tvec=tvec[i], length=0.1);
                    # markerLength = 0.1
                    # m = markerLength / 2
                    # pts_0 = np.float32([[-m, m, 0], [m, m, 0], [-m, -m, 0], [-m, m, m]])
                    # pt_dict = {}
                    # imgpts, _ = cv2.projectPoints(pts_0, rvec, tvec, self.rgtmtx, self.rgtdist)
                    # for i in range(len(pts_0)):
                    #     pt_dict[tuple(pts_0[i])] = tuple(imgpts[i].ravel())
                    # src = pt_dict[tuple(pts_0[0])]
                    # dst1 = pt_dict[tuple(pts_0[1])]
                    # dst2 = pt_dict[tuple(pts_0[2])]
                    # dst3 = pt_dict[tuple(pts_0[3])]
                    #
                    # cv.line(frame, src, dst1, (0, 255, 0), 4)
                    # cv.line(frame, src, dst2, (255, 0, 0), 4)
                    # cv.line(frame, src, dst3, (0, 0, 255), 4)
                for i in range(ids.shape[0]):

                    if ids[i] == marker_id:
                        det_rvec = rvec[i]  # check this if ERROR
                        # print( "------------------------det_rvec------\n",det_rvec)
                        det_tvec = 1000 * tvec[i]  # check this if ERROR
                        det_rmat, _ = cv2.Rodrigues(det_rvec)
                        det_T_MC[:3, :3] = det_rmat
                        det_T_MC[:3, 3] = det_tvec[0]
                        T_MW_a = np.dot(T_CW_a, det_T_MC)  # ought to be the left hand pose
                        detected = True
                        break
                if detected:
                    break
        final_T = T_MW_a
        # print(final_T)

        # base.pggen.plotAxis(base.render, spos=final_T[:3, 3], pandamat4=base.pg.np4ToMat4(final_T))

        return frame, final_T


def getObjfromCam(self,  eepos, eerotmat, marker_id=1):
    """
    get the homo pose of obj using rgtcam

    :param eepos: endeffector pose
    :param eerotmat:  end effector rotmat
    :return:

    author: weiwei, mmd
    date: 20180925
    """

    # hcc = cv.VideoCapture(0)
    frame = self.getlc0img()
    # connectToCam = rpyc.connect("10.2.0.60", 18300)
    # frame = pickle.loads(connectToCam.root.exposed_getrc0img())
    # # frame = cv2.flip(frame,-1)
    # fopen = open('photoNew.pickle', 'wb')
    # pickle.dump(frame, fopen)
    # fopen.close()

    rgtarmrot = eerotmat
    rgtarmpos = eepos
    T_RGTW_a = np.eye(4)
    T_RGTW_a[:3, :3] = rgtarmrot
    T_RGTW_a[:3, 3] = rgtarmpos
    T_CW_a = np.dot(T_RGTW_a, self.T_CR_rgt)
    det_T_MC = np.eye(4)



    # base.pggen.plotSphere(base.render, pos=T_CW_a[:3, 3], radius=20)
    # base.pggen.plotAxis(base.render, spos=T_CW_a[:3, 3], pandamat4=base.pg.np4ToMat4(T_CW_a))


    T_MW_a = None
    for i in range(1):
        detected = False
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if np.all(ids != None):
            pts = np.array(corners[0],dtype = np.int32)
            cv.polylines(img=frame,pts=pts, isClosed=True, color=(255, 0, 0), thickness=5, lineType=8, shift=0)

            ct = np.sum(corners[0], axis=1) / corners[0].shape[1]
            ct.astype(int)
            ct = tuple(ct[0])
            cv.circle(img=frame, center=ct, radius=100, color=(0, 255, 0), thickness=5, lineType=8, shift=0)

            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.035, self.rgtmtx,
                                                            self.rgtdist)  # the second value is the marker size in meters
            print("------------------------rvec------\n", rvec,type(rvec), "\n","------------------------rvec------\n", )
            print("------------------------tvec------\n", tvec, type(tvec), "\n",
                  "------------------------tvec------\n", )
            for i in range(ids.shape[0]):

                cv.aruco.drawAxis(image=frame, cameraMatrix=self.rgtmtx, distCoeffs=self.rgtdist, rvec=rvec[i], tvec=tvec[i], length=0.1);
                # markerLength = 0.1
                # m = markerLength / 2
                # pts_0 = np.float32([[-m, m, 0], [m, m, 0], [-m, -m, 0], [-m, m, m]])
                # pt_dict = {}
                # imgpts, _ = cv2.projectPoints(pts_0, rvec, tvec, self.rgtmtx, self.rgtdist)
                # for i in range(len(pts_0)):
                #     pt_dict[tuple(pts_0[i])] = tuple(imgpts[i].ravel())
                # src = pt_dict[tuple(pts_0[0])]
                # dst1 = pt_dict[tuple(pts_0[1])]
                # dst2 = pt_dict[tuple(pts_0[2])]
                # dst3 = pt_dict[tuple(pts_0[3])]
                #
                # cv.line(frame, src, dst1, (0, 255, 0), 4)
                # cv.line(frame, src, dst2, (255, 0, 0), 4)
                # cv.line(frame, src, dst3, (0, 0, 255), 4)
            for i in range(ids.shape[0]):

                if ids[i] == marker_id:
                    det_rvec = rvec[i]  # check this if ERROR
                    # print( "------------------------det_rvec------\n",det_rvec)
                    det_tvec = 1000 * tvec[i]  # check this if ERROR
                    det_rmat, _ = cv2.Rodrigues(det_rvec)
                    det_T_MC[:3, :3] = det_rmat
                    det_T_MC[:3, 3] = det_tvec[0]
                    T_MW_a = np.dot(T_CW_a, det_T_MC)  # ought to be the left hand pose
                    detected = True
                    break
            if detected:
                break
    final_T = T_MW_a
    # print(final_T)

    # base.pggen.plotAxis(base.render, spos=final_T[:3, 3], pandamat4=base.pg.np4ToMat4(final_T))

    return frame


import time
import utiltools.robotmath as ur
import math
def getVec():
    hdc = HndCam()
    time_zero = time.time()
    # print(time_zero, "time_zero")
    filename="a"
    video = cv2.VideoCapture(1)
    vec_xlist = []
    vec_ylist = []
    degree_check = []
    count = 10
    i = 0
    while i < count:
        ret, frame = video.read()
        # cv2.imshow('frame', frame)
        dnpa_get = hdc.getObjfromVideo(frame=frame, eepos=[0, 0, 0], eerotmat=np.eye(3), marker_id=0)
        # dnpa = dnpa_get[0]
        tmat = dnpa_get[1]
        # print(tmat, "tmat")
        if np.all(tmat == None):
            continue
        else:
            vec_xlist.append(tmat[:3, 0])
            vec_ylist.append(tmat[:3, 1])
            i += 1
        # np.cross(np.array(vec_ylist[0]), np.array(vec_ylist[i]))
        # cv2.imshow("Depth", dnpa)
        # cv2.imwrite('test_fromvideo10.jpg', dnpa)
        # cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vec = np.average(np.asarray(vec_ylist, dtype=object), axis=0)
    # with open(filename + '.pickle', 'wb') as f:
    #     pickle.dump(vec, f)
    video.release()
    cv2.destroyAllWindows()
    return vec

    # start = getVec()

    # while True:
    #     current_vec = getVec()
    #     angle = rm.degree_betweenvector(start, current_vec)
    #     print(angle)
    #     time.sleep(1)
