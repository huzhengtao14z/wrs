import itertools
import os
import visualization.panda.world as wd
import numpy as np
import basis.robot_math as rm
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.core import *
from shapely.geometry import Point
from shapely.geometry import Polygon
from sklearn.cluster import KMeans
from sklearn.neighbors import RadiusNeighborsClassifier
import modeling.collision_model as cm
import modeling.geometric_model as gm
# import pandaplotutils.pandactrl as pandactrl
# import pandaplotutils.pandageom as pandageom
# import trimesh.sample as sample
# import sys
# import trimesh
# from utiltools import robotmath
# import pickle
import basis.trimesh.sample as sample
import basis.trimesh as trimesh
import time
from hu import humath
# from hu import hufunc
import random
# import utiltools.misc.p3dutils as p3du
# import utiltools.robotmath as rm

class FreeholdContactpairs(object):

    def __init__(self, objpath, faceangle=.99, segangle=.99, refine1min=2, refine1max=30,
                 refine2radius=10, fpairparallel=-0.7, hmax=5, objmass=20.0, bypasssoftfgr=True, useoverlap=True):
        """
        :param objpath: path of the object
        :param hand: An object of a 2F hand class
        :param faceangle: The threhold angle for two triangles to be considered as co-planer
        :param segangle: The threhold angle for two facets to be considered as co-planer
        :param refine1min: The minimum distance between a contact point and the facet boundary
        :param refine1max: The maximum distance between a contact point and the facet boundary
        :param refine2radius: Size of the contact circle
        :param fpairparallel: The threhold dot product for two facets to be considered as parallel
        :param hmax: The maximum punction depth of a soft finger contact model
        :param objmass: Mass of the object
        :param bypasssoftfgr: Boolean value to switch on the soft finger model. True means it is switched off by default
        :param useoverlap: for comparison, toggle it off if overlaped segments are needed

        author: hu
        date: 202203osaka
        """

        self.objcm = cm.CollisionModel(objpath)
        self.objtrimesh = self.objcm.objtrm #trimesh the STL files\
        self.objcm.set_rgba((.5, .7, .3, 0.3))
        self.objcm.set_scale((0.001,0.001,0.001))
        # self.objcm.attach_to(base)
        # gm.gen_frame().attach_to(self.objcm)
        print("check faces","there are",len(self.objtrimesh.faces),self.objtrimesh.faces)
        print("check facets", "there are",len(self.objtrimesh.facets()),self.objtrimesh.facets())
        # generate facets
        tic = time.time()
        if useoverlap: #(trimesh.facets_over)Compute facets using oversegmentation
            self.facets, self.facetnormals, self.facetcurvatures = self.objtrimesh.facets_over(face_angle=faceangle, seg_angle=segangle)
        else:
            self.facets, self.facetnormals, self.facetcurvatures = self.objtrimesh.facets_noover(faceangle = faceangle)
        toc = time.time()
        print("facet cost", toc -tic)

        self.vertices=self.objtrimesh.vertices
        self.smallfaces = self.objtrimesh.faces

        # the sampled points and their normals
        tic = time.time()
        self.objsamplepnts = None
        self.objsamplenrmls = None
        # self.sampleObjModel()
        toc = time.time()
        print("sampling cost", toc-tic)

        # the sampled points (bad samples removed)
        tic = time.time()
        # facet2dbdries saves the 2d boundaries of each facet
        # self.removeBadSamples(mindist=refine1min, maxdist=refine1max)
        toc = time.time()
        print("remove bad sample cost", toc-tic)

        # the sampled points (clustered)
        tic = time.time()
        # self.clusterFacetSamplesRNN(reduceRadius=refine2radius)
        toc = time.time()
        print("cluster samples cost", toc-tic)

        self.faces = self.objtrimesh.faces
        self.com = self.objtrimesh.center_mass
        self.facesarea = []
        self.largefacetsarea = []

        self.smallfacecenter=[]
        self.largefacescenter = []

        self.largeface_normals = []
        self.largeface_vertices = []
        # plan contact pairs
        self.holdpairs = []
        self.holdpairsnum = None
        self.holdingpairsnormals = []
        self.holdingpairscenters = []
        self.holdingpairsvertices = []
        self.holdingpairsCoM = []
        self.holdingpairsfaces = []
        self.holdingpairsfaces_triple = []

        self.placementRotMat = []
        self.placementNormals = []
        self.placementfacetpairscenter = []
        self.placementVertices = []
        self.placementCoM = []
        self.placementfaces = []

        self.color = (random.random(), random.random(), random.random(), 1)
        tic = time.time()
        # self.planContactpairs(hmax, fpairparallel, objmass, bypasssoftfgr = bypasssoftfgr)
        toc = time.time()
        print("plan contact pairs cost", toc-tic)

        # for plot
        self.counter = 0
        # self.facetcolorarray = pandageom.randomColorArray(self.facets.shape[0], nonrandcolor = [.5,.5,.7,1])

    def sampleObjModel(self, numpointsoververts=5):
        """
        sample the object model
        self.objsamplepnts and self.objsamplenrmls
        are filled in this function

        :param: numpointsoververts: the number of sampled points = numpointsoververts*mesh.vertices.shape[0]
        :return: nverts: the number of verts sampled

        author: weiwei
        date: 20160623 flight to tokyo
        """

        nverts = self.objtrimesh.vertices.shape[0]
        samples, face_idx = sample.sample_surface_even(self.objtrimesh,
                                                       count=(1000 if nverts*numpointsoververts > 1000 \
                                                                  else nverts*numpointsoververts))
        # print nverts
        self.objsamplepnts = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        for i, faces in enumerate(self.facets):
            for face in faces:
                sample_idx = np.where(face_idx==face)[0]
                if len(sample_idx) > 0:
                    if self.objsamplepnts[i] is not None:
                        self.objsamplepnts[i] = np.vstack((self.objsamplepnts[i], samples[sample_idx]))
                        self.objsamplenrmls[i] = np.vstack((self.objsamplenrmls[i],
                                                            [self.objtrimesh.face_normals[face]]*samples[sample_idx].shape[0]))
                    else:
                        self.objsamplepnts[i] = np.array(samples[sample_idx])
                        self.objsamplenrmls[i] = np.array([self.objtrimesh.face_normals[face]]*samples[sample_idx].shape[0])
            if self.objsamplepnts[i] is None:
                self.objsamplepnts[i] = np.empty(shape=[0,0])
                self.objsamplenrmls[i] = np.empty(shape=[0,0])
        return nverts

    def drawSingleFaceSurface(self, base, vertices, faces, color):
        '''
        draw a surface using a calculated fourth point to creat a hull
        :param base:
        :param vertices:
        :param faces:
        :param color:
        :return:
        '''
        # print("faces in plotsurface",faces)

        surface_vertices = np.array([vertices[faces[0]], vertices[faces[1]], vertices[faces[2]]])
        surface = humath.centerPoftrangle(surface_vertices[0], surface_vertices[1], surface_vertices[2])
        surface = trimesh.Trimesh(surface)
        surface = surface.convex_hull
        surface = gm.GeometricModel(surface)
        surface.set_rgba(color)
        surface.attach_to(base)

    def showFacetsPair(self, base, id):
        vertices = self.vertices
        surface_id_1 = self.holdpairs[id][0]
        surface_id_2 = self.holdpairs[id][1]
        color = (random.random(), random.random(), random.random(), 1)
        face_1 = self.facets[surface_id_1]
        face_2 = self.facets[surface_id_2]
        print(face_1, face_2,"hi")
        for face in face_1:
            self.drawSingleFaceSurface(base, vertices, self.objtrimesh.faces[face], color=color)
        for face in face_2:
            self.drawSingleFaceSurface(base, vertices, self.objtrimesh.faces[face], color=color)
        # for i,facets in enumerate(self.facets):
        #     color = (random.random(),random.random(),random.random(),0.1)
        #     for facet in facets:
        #         hufunc.drawSingleFaceSurface(base,vertices,self.objtrimesh.faces[facet],color=color)

    def planHoldpairs(self, verticaljudge_lft=-np.cos(np.pi * 0.5 * 0.95),
                      verticaljudge_rgt=np.cos(np.pi * 0.5 * 0.95)):
        '''
        find the holding pairs considering a jig with 3 mutually perpendicular surface
        :param verticaljudge_lft:
        :param verticaljudge_rgt:

        :return:

        author: weiwei, hu zhengtao
        date: 2020/04/03 osaka university
        '''

        # facetparis for update
        updatedholdingfacetpairs = []

        # find facets combinations (3)
        facets_num = self.facets.shape[0]
        self.temholdingpairs = list(itertools.combinations(range(facets_num), 2))

        for facetpair in self.temholdingpairs:
            temppairscenter = []
            tempgripcontactpairnormals = []
            dotnorm = np.dot(self.facetnormals[facetpair[0]], self.facetnormals[facetpair[1]])
            diff_angle = rm.angle_between_vectors(self.facetnormals[facetpair[0]], self.facetnormals[facetpair[1]])
            if diff_angle > np.radians(58) and diff_angle < np.radians(62):
            # if dotnorm > verticaljudge_lft and dotnorm < verticaljudge_rgt:
                updatedholdingfacetpairs.append(facetpair)
        self.holdpairs = updatedholdingfacetpairs
        self.holdpairsnum = len(self.holdpairs)

    def getCoordinate(self):
        coordinata = []
        doubleholdpair = []
        for pair in self.holdpairs:
            normal_0 = -self.facetnormals[pair[0]]
            normal_1 = -self.facetnormals[pair[1]]
            # normal_1 = normal_1, rm.rotmat_from_axangle()
            normal_2a = np.cross(normal_0, normal_1)
            normal_2b = np.cross(normal_1, normal_0)
            normal_0 = np.dot( rm.rotmat_from_axangle(normal_2a, np.radians(-15)),normal_0)
            normal_1 = np.dot(rm.rotmat_from_axangle(normal_2a, np.radians(15)),  normal_1)
            coordinate0 = np.array([normal_0, normal_1, normal_2a])
            coordinate1 = np.array([normal_1, normal_0, normal_2b])
            coordinata.append(coordinate0.T)
            coordinata.append(coordinate1.T)
            doubleholdpair.append(pair)
            doubleholdpair.append(pair)
        self.coordinate = coordinata
        self.doubleholdpair = doubleholdpair
        self.gettwoend()

    def gettwoend(self):
        self.endpairs=[]
        self.temporigin = []
        self.origin=[]
        self.homomat=[]
        for i, coordinate in enumerate(self.coordinate):
            endpair = []
            axis = coordinate.T[2]
            large = 0
            large_vec = np.array([0,0,0])
            small = 0
            small_vec = np.array([0, 0, 0])
            facet_1_ID=self.doubleholdpair[i][0]
            facet_2_ID = self.doubleholdpair[i][1]
            vertices = np.append(self.largeface_vertices[facet_1_ID], self.largeface_vertices[facet_2_ID], axis = 0)

            for vertice in vertices:
                value = np.dot(axis, vertice)
                if value>=large:
                    large = value
                    large_vec = vertice
                elif value<small:
                    small = value
                    small_vec = vertice
            endpair.append(large_vec)
            endpair.append(small_vec)
            self.temporigin.append((large_vec+small_vec)*0.5)
            self.endpairs.append(endpair)
            origin = self.getorigin(normal_list=[self.largeface_normals[facet_1_ID],self.largeface_normals[facet_2_ID], axis], point_list=[self.largefacescenter[facet_1_ID],self.largefacescenter[facet_2_ID],(large_vec+small_vec)*0.5])
            self.origin.append(origin)
            homomat = np.linalg.inv(rm.homomat_from_posrot(origin, coordinate))
            self.placementRotMat.append(homomat)
            self.placementCoM.append(rm.homomat_transform_points(homomat, self.com))
            tempvertice = []
            for vertice in vertices:
                tempvertice.append(rm.homomat_transform_points(homomat, vertice))
            self.placementVertices.append(tempvertice)
            self.placementNormals.append([np.dot(coordinate.T, self.largeface_normals[facet_1_ID]),np.dot(coordinate.T, self.largeface_normals[facet_2_ID])])
            self.placementfacetpairscenter.append([rm.homomat_transform_points(homomat,self.largefacescenter[facet_1_ID]), rm.homomat_transform_points(homomat,self.largefacescenter[facet_2_ID])])

    def getFacetsCenter(self):

        #get the coordinate of large facet center in self.largefacescenter
        #get the normal vecter of large facet center in self.largeface_normals
        #get the vertices coordinate of large facet in self.largeface_vertices

        self.facesarea = self.objtrimesh.area_faces
        self.smallface_normals = self.objtrimesh.face_normals

        for i,smallface in enumerate(self.objtrimesh.faces):
            self.smallfacecenter.append(humath.centerPoint(np.array([self.objtrimesh.vertices[smallface[0]],
                                                                 self.objtrimesh.vertices[smallface[1]],
                                                                 self.objtrimesh.vertices[smallface[2]]])))
        self.largeface = self.facets
        self.prelargeface = self.facets
        smallfacectlist = []
        smallfacectlist_area = []
        for i in range(len(self.largeface)):
            b = []
            b_area = []
            temlargefaceVerticesid = []
            temlargefaceVertices = []
            for j in range(len(self.largeface[i])):
                b.append(self.smallfacecenter[self.largeface[i][j]])
                b_area.append(self.facesarea[self.largeface[i][j]])
                temlargefaceVerticesid.extend(self.smallfaces[self.largeface[i][j]])
                print("temlargefaceVerticesid",temlargefaceVerticesid)
            smallfacectlist.append(b)
            smallfacectlist_area.append(b_area)
            self.largeface_normals.append(self.smallface_normals[self.largeface[i][0]])
            temlargefaceVerticesid = list(set(temlargefaceVerticesid)) #cancel repeating vertices ID
            for id in temlargefaceVerticesid:
                temlargefaceVertices.append(self.vertices[id])
            self.largeface_vertices.append(temlargefaceVertices)

        for i,largeface in enumerate(smallfacectlist):
            self.largefacescenter.append(humath.centerPointwithArea(largeface,smallfacectlist_area[i]))

    def getorigin(self, normal_list, point_list):
        surface_A=[]
        surface_B = np.zeros(shape=(3))
        normal_0 = normal_list[0]
        normal_1 = normal_list[1]
        normal_2 = normal_list[2]
        pnt_0 = point_list[0]
        pnt_1 = point_list[1]
        pnt_2 = point_list[2]
        surface_A.append(normal_0)
        surface_A.append(normal_1)
        surface_A.append(normal_2)
        surface_A = np.array(surface_A)
        surface_B[0] = np.dot(normal_0, pnt_0)
        surface_B[1] = np.dot(normal_1, pnt_1)
        surface_B[2] = np.dot(normal_2, pnt_2)
        return np.linalg.solve(surface_A, surface_B)


    def showCoodinate(self, id):

        axis1 = self.coordinate[2*id]
        # print("check1")
        # print(axis1, "11")
        gm.gen_mycframe(rotmat=axis1, pos=self.origin[2*id],  thickness=0.01).attach_to(base)
        # axis2 = self.coordinate[2*id+1]
        # gm.gen_frame(rotmat=axis2).attach_to(base)

        print(self.endpairs[2*id][0],"cccc")
        # gm.gen_sphere(pos=self.endpairs[2*id][0], radius=0.0051).attach_to(base)
        # gm.gen_sphere(pos=self.endpairs[2 * id][1], radius=0.0051).attach_to(base)
        # gm.gen_sphere(pos=self.temporigin[2 * id], radius=0.01).attach_to(base)
        gm.gen_sphere(pos=self.origin[2 * id], radius=0.01).attach_to(base)

if __name__=='__main__':
    import os.path

    this_dir, this_filename = os.path.split(__file__)
    base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960, h=540, lookat_pos=[0, 0, 0.0])
    # gm.gen_frame(length=1).attach_to(base)
    # base = wd.World(cam_pos=[600, 300, 900], w=960, h=540, lookat_pos=[0, 0, 0.0])
    a = np.array([[-1, -0, 0.],[-0, -0, -1.],[-0, -1, 0]])
    # gm.gen_frame(rotmat=a).attach_to(base)
    objpath = os.path.join(this_dir, "objects", "l-cylinder.STL")
    freehold = FreeholdContactpairs(objpath)
    freehold.planHoldpairs()
    # freehold.showFacetsPair(base = base, id =0)
    freehold.getFacetsCenter()
    freehold.getCoordinate()
    freehold.gettwoend()
    # freehold.showCoodinate(id =0)

    fixturepath = os.path.join(this_dir, "objects", "v120.STL")
    fixturecm = cm.CollisionModel(fixturepath)
    fixturecm.set_rgba((1, 1, .1, 1))
    fixturecm.set_scale((0.001, 0.001, 0.001))
    fixturecm.set_pos((-0.002, -0.002, -0.050))
    fixturecm.set_rpy(0, 0, np.radians(-45))
    fixturecm.attach_to(base)

    from direct.gui.OnscreenText import OnscreenText

    counter=[0]
    obj_show_node = [None,None,None,None] #0Mesh, 1CoM, 2Normal
    def update(textNode, obj_show_node, counter, task):
        if base.inputmgr.keymap['space'] is True:
            print("space")
            # time.sleep(0.5)
            if counter[0] >= len(freehold.placementRotMat):
                counter[0] = 0
            if obj_show_node[0] is not None:
                # obj_show_node[0].detach()
                obj_show_node[1].detachNode()
                obj_show_node[2].detachNode()
                obj_show_node[3].detachNode()
            obj_show_node[0] = cm.CollisionModel(objpath)
            obj_show_node[0].set_scale((0.001, 0.001, 0.001))
            obj_show_node[0].set_rgba((0, 191 / 255, 1, 0.5))
            obj_show_node[0].set_homomat(freehold.placementRotMat[counter[0]])

            if obj_show_node[0].is_mcdwith(fixturecm):
                obj_show_node[0].set_rgba((1, 0, 0, 0.5))
            else:
                obj_show_node[0].attach_to(base)
            obj_show_node[1] = NodePath("com")
            obj_show_node[2] = NodePath("vertice")
            obj_show_node[3] = NodePath("normal")
            # gm.gen_sphere(pos = freehold.placementCoM[counter[0]]).attach_to(obj_show_node[1])
            obj_show_node[1].reparentTo(base.render)
            # for vertace in freehold.placementVertices[counter[0]]:
            #     gm.gen_sphere(pos=vertace, radius=0.005).attach_to(obj_show_node[2])
            obj_show_node[2].reparentTo(base.render)
            for i, normal in enumerate(freehold.placementNormals[counter[0]]):
                print(normal)
                gm.gen_arrow(spos=freehold.placementfacetpairscenter[counter[0]][i], epos=freehold.placementfacetpairscenter[counter[0]][i]+normal*0.1).attach_to(obj_show_node[3])
            obj_show_node[3].reparentTo(base.render)
            counter[0]+=1


        if textNode[0] is not None:
            textNode[0].detachNode()
            textNode[1].detachNode()
            textNode[2].detachNode()
        cam_pos = base.cam.getPos()
        textNode[0] = OnscreenText(
            text=str(cam_pos[0])[0:5],
            fg=(1, 0, 0, 1),
            pos=(1.0, 0.8),
            align=TextNode.ALeft)
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
    taskMgr.doMethodLater(0.01, update, "addobject", extraArgs=[testNode,obj_show_node, counter], appendTask=True)
    base.run()
