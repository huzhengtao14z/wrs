import copy
import math
import visualization.panda.world as wd
import modeling.collision_model as cm
import humath as hm
import hufunc as hf
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode
import numpy as np
import basis.robot_math as rm
import modeling.geometric_model as gm
import os
import pickle
import basis.data_adapter as da
import slope
import Sptpolygoninfo as sinfo
import basis.trimesh as trimeshWan
import trimesh as trimesh
from trimesh import ray as triray
from trimesh.sample import sample_surface
from panda3d.core import NodePath
import trimeshwraper as tw

import open3d as o3d
# import open3d.geometry as o3dg
import vision.depth_camera.pcd_data_adapter as vdda



if __name__ == '__main__':
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)
    gm.gen_frame()
    viewball = gm.gen_sphere(radius=0.3, rgba=(1,0,0,0.1), subdivisions=0)
    viewball.attach_to(base)
    viewpointlist = viewball.objtrm.vertices
    print(viewpointlist)
    for point in viewpointlist:
        gm.gen_sphere(point, radius=0.03, rgba=(0,1,0,1)).attach_to(base)
    objdir = "../kit_model"
    namelist = os.listdir(objdir)
    name = "Amicelli_800_tex"

    # mesh = cm.CollisionModel(objdir+"/"+name+".obj")
    # mesh.attach_to(base)

    obj = tw.TrimeshHu("../kit_model/", name+".obj", scale=1)
    mesh = obj.outputTrimesh
    testmesh = gm.GeometricModel(mesh)
    testmesh.set_rgba([1, 0, 0, 1])
    # testmesh.attach_to(base)

    intersector = triray.ray_pyembree.RayMeshIntersector(mesh)

    faces = mesh.faces
    vertices = mesh.vertices
    check_list = []
    origin_list = []
    origin = viewpointlist[0]
    for face in faces:
        points = [vertices[face[0]], vertices[face[1]], vertices[face[2]]]
        direction = np.array(hm.centerPoint(points) - origin)
        check_list.append(direction)
        origin_list.append(origin)
    viewed = intersector.intersects_first(ray_origins=origin_list,
                                          ray_directions=check_list)
    viewed = hm.listnorepeat(viewed)

    # for item in viewed_faces:
    #     hf.drawanySingleSurface(base, vertices = [vertices[item[0]],vertices[item[1]],vertices[item[2]],hm.centerPoint([vertices[item[0]],vertices[item[1]],vertices[item[2]]])], color=(0,1,0,1))

    viewed_faces = [faces[i] for i in viewed]
    list_viewedvertexid = list(set(np.asarray(viewed_faces).flatten().tolist()))


    def updateid(idlist, face):
        newid0 = idlist.index(face[0])
        newid1 = idlist.index(face[1])
        newid2 = idlist.index(face[2])
        return [newid0, newid1, newid2]


    viewed_vertices = []
    for item in list_viewedvertexid:
        viewed_vertices.append(vertices[item])

    viewed_faces = [updateid(list_viewedvertexid, faces[i]) for i in viewed]
    # for item in viewed_faces:
    #     hf.drawanySingleSurface(base, vertices = [viewed_vertices[item[0]],viewed_vertices[item[1]],viewed_vertices[item[2]],hm.centerPoint([viewed_vertices[item[0]],viewed_vertices[item[1]],viewed_vertices[item[2]]])], color=(0,1,0,1))

    viewedmesh = trimesh.Trimesh(vertices=viewed_vertices, faces=viewed_faces)

    viewedmesh.export("testtest.stl")
    test = gm.GeometricModel("testtest.stl")
    test.set_rgba([0, 1, 0, 1])
    test.attach_to(base)

    base.run()

    n = str(1)
    # pcd = o3d.io.read_point_cloud('./pairtial/pairtial_pc/'+name+n+".ply")
    pcd = o3d.io.read_point_cloud('./pairtial_pc/' + name + n + ".ply")
    pcd.paint_uniform_color([0,0,0.5])
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    normals = np.asarray(pcd.normals)
    print(normals)
    # o3d.visualization.draw_geometries([pcd])

    pcd_list = vdda.o3dpcd_to_parray(pcd)

    with open('pairtial/com_v2.pickle', 'wb') as f:
        com_list = pickle.load(f)
    com = com_list[name+n]
    gm.gen_sphere(com).attach_to(base)
    gm.gen_pointcloud(pcd_list).attach_to(base)
    base.run()


    def apply_noise(pcd, mu, sigma):
        noisy_pcd = copy.deepcopy(pcd)
        points = np.asarray(noisy_pcd.points)
        points += np.random.normal(mu, sigma, size=points.shape)
        noisy_pcd.points = o3d.utility.Vector3dVector(points)
        return noisy_pcd

    mu, sigma = 0, 0.001  # mean and standard deviation
    source_noisy = apply_noise(pcd, mu, sigma)
    o3d.visualization.draw_geometries([source_noisy])
    print("Source PointCloud + noise:")

    def update(textNode, count, task):

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
    count = [0]
    taskMgr.doMethodLater(0.01, update, "addobject", extraArgs=[testNode, count],
                          appendTask=True)

    base.run()