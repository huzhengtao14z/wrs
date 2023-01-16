import copy
import math
import visualization.panda.world as wd
import modeling.collision_model as cm
import robot_sim.end_effectors.grippers.yumi_gripper.yumi_gripper as yg
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as hnde
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode
import numpy as np
import basis.robot_math as rm
import modeling.geometric_model as gm
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_sim.robots.ur3e_dual.ur3e_dual as ur3ed
import robot_sim.robots.sda5f.sda5f as sda5
import motion.probabilistic.rrt_connect as rrtc
import manipulation.pick_place_planner as ppp
import os
import pickle
import basis.data_adapter as da
import slope
import Sptpolygoninfo as sinfo
import basis.trimesh as trimeshWan
import trimesh as trimesh
from trimesh.sample import sample_surface
from panda3d.core import NodePath
import trimeshwraper as tw
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.robotiq85.robotiq85 as rtq85
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe
import open3d as o3d
import vision.depth_camera.pcd_data_adapter as vdda
import humath as ma

def apply_noise(pcd, mu, sigma):
    noisy_pcd = copy.deepcopy(pcd)
    points = np.asarray(noisy_pcd.points)
    points += np.random.normal(mu, sigma, size=points.shape)
    noisy_pcd.points = o3d.utility.Vector3dVector(points)
    return noisy_pcd

def uniform_random_rotate():
    x0 = np.random.random()
    y1 = 2*math.pi*np.random.random()
    y2 = 2*math.pi*np.random.random()
    r1 = math.sqrt(1.0-x0)
    r2 = math.sqrt(x0)
    u0 = math.cos(y2)*r2
    u1 = math.sin(y1)*r1
    u2 = math.cos(y1)*r1
    u3 = math.sin(y2)*r2
    coefi = 2.0*u0*u0-1.0
    coefuu = 2.0
    coefe = 2.0*u0
    r = np.ndarray(shape=(3, 3))
    r[0, 0] = coefi+coefuu*u1*u1
    r[1, 1] = coefi+coefuu*u2*u2
    r[2, 2] = coefi+coefuu*u3*u3

    r[1, 2] = coefuu*u2*u3-coefe*u1
    r[2, 0] = coefuu*u3*u1-coefe*u2
    r[0, 1] = coefuu*u1*u2-coefe*u3

    r[2, 1] = coefuu*u3*u2+coefe*u1
    r[0, 2] = coefuu*u1*u3+coefe*u2
    r[1, 0] = coefuu*u2*u1+coefe*u3

    # r = np.insert(r, 3, 0, axis=0)
    # r = np.insert(r, 3, 0, axis=1)
    # r[3, 3] = 1
    return r

if __name__ == '__main__':
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)
    adress = "cylinder/"
    files = os.listdir(adress)
    comdict = {}
    #files = ["Amicelli_800_tex.obj", "BakingSoda_800_tex.obj"]
    # print(files)
    # base.run()
    # name = "mug"
    # name = "airplaneremesh"
    from collections import defaultdict
    profile = defaultdict(list)
    for names in files:
        name = names
        print(name)
        # name = "armadillo"
        # name = "mug"
        obj = tw.TrimeshHu(meshpath=adress, name=name, scale=0.001)
        mesh = obj.outputTrimesh
        #####重心
        d = cm.CollisionModel(mesh)
        d = d.get_com()
        # testmesh = gm.GeometricModel(mesh)
        # testmesh.set_rgba([1, 0, 0, 1])
        # testmesh.attach_to(base)


        icosphere = gm.gen_sphere(radius=0.15, rgba=[0, 0, 1, 0.1], subdivisions=0)
        sample = icosphere.objtrm.vertices
        # profile[name].append(sample)
        # profile[name].append(d)
        count = 0
        for origin in sample:
            intersector = trimesh.base.ray.ray_pyembree.RayMeshIntersector(mesh)
            import humath as hm
            import hufunc as hf
            faces = mesh.faces
            vertices = mesh.vertices
            check_list=[]
            origin_list=[]
            for face in faces:
                points = [vertices[face[0]], vertices[face[1]], vertices[face[2]]]
                direction = np.array(hm.centerPoint(points)-origin)
                check_list.append(direction)
                origin_list.append(origin)
            viewed = intersector.intersects_first(ray_origins=origin_list,
                                 ray_directions=check_list)
            viewed=hm.listnorepeat(viewed)


            # for item in viewed_faces:
            #     hf.drawanySingleSurface(base, vertices = [vertices[item[0]],vertices[item[1]],vertices[item[2]],hm.centerPoint([vertices[item[0]],vertices[item[1]],vertices[item[2]]])], color=(0,1,0,1))

            viewed_faces = [faces[i] for i in viewed]
            list_viewedvertexid = list(set(np.asarray(viewed_faces).flatten().tolist()))
            def updateid(idlist,face):
                newid0 = idlist.index(face[0])
                newid1 = idlist.index(face[1])
                newid2 = idlist.index(face[2])
                return [newid0,newid1,newid2]

            viewed_vertices = []
            for item in list_viewedvertexid:
                    viewed_vertices.append(vertices[item])

            viewed_faces = [updateid(list_viewedvertexid,faces[i]) for i in viewed]
            # for item in viewed_faces:
            #     hf.drawanySingleSurface(base, vertices = [viewed_vertices[item[0]],viewed_vertices[item[1]],viewed_vertices[item[2]],hm.centerPoint([viewed_vertices[item[0]],viewed_vertices[item[1]],viewed_vertices[item[2]]])], color=(0,1,0,1))

            viewedmesh=trimesh.Trimesh(vertices = viewed_vertices, faces=viewed_faces)

            viewedmesh.export("testtest.stl")
            test=gm.GeometricModel("testtest.stl")
            test.set_rgba([1,0,0,1])
            #test.attach_to(base)

            Mesh = o3d.io.read_triangle_mesh("testtest.stl")
            Mesh.compute_vertex_normals()
            pcd1 = Mesh.sample_points_poisson_disk(number_of_points = 500)
            pcd1_np=vdda.o3dpcd_to_parray(pcd1)


            #r=uniform_random_rotate()
            #print(r)
            #pcd1_rot_np = np.dot(pcd1_np, r)
            # pcd1_rot=copy.deepcopy(pcd1)
            # pcd1_rot.rotate(r, center=(0,0,0))
            #o3d.visualization.draw_geometries([pcd1, pcd1_rot])

            #pcd1_rot_np = vdda.o3dpcd_to_parray(pcd1_rot)
            #pcd1_rot_np_v = gm.GeometricModel(pcd1_rot_np).attach_to(base)
            #print(pcd1_np.shape)
            #print(pcd1_np)
            #pcd1_v = gm.GeometricModel(pcd1_np).attach_to(base)
            #print(pcd1_rot_np)
            center=np.average(pcd1_np,axis = 0)
            tm = np.linalg.inv(rm.homomat_from_posrot(center))
            pcd1.transform(tm)
            com = rm.homomat_transform_points(tm, d)
            # gm.gen_sphere(d,rgba=(0,1,0,1)).attach_to(base)
            # gm.gen_sphere(com).attach_to(base)
            # pcd1_np_v = gm.GeometricModel(pcd1_rot_np).attach_to(base)
            #base.run()
            # mu, sigma = 0, 0.001  # mean and standard deviation
            #
            # source_noisy = apply_noise(pcd1, mu, sigma)

            filename = ''
            objname = name
            objname = objname.replace('.stl', '')
            filename = objname + str(count) +".ply"
            comname = objname + str(count)
            comdict[comname] = com
            # filename2 = filename = objname + str(count) +".stl"
            #filename3 = "noizy" + objname + str(count) + ".ply"
            # adress_pc = ""
            #adress_pc = "pirtial_pc/"+filename
            # adress_mesh=""
            # adress_mesh= "partial_mesh/" + filename2
            adress_pc = "pc_cylinder/" + filename

            o3d.io.write_point_cloud(adress_pc, pcd1)
            #viewedmesh.export(adress_pc)
            #ptCloud = o3d.io.read_point_cloud(adress_pc)
            #print(ptCloud)
            count = count+1


    with open('cc_com.pickle', 'wb') as file:
        pickle.dump(comdict, file)
    print('finish')
    base.run()
    #
    #
    #
    #with open('com_cc.pickle', 'rb') as file:
    #     print(file)
    sample, _= sample_surface(obj_pcd, 2048, sample_color=False)
    pcd_list = trimesh.points.PointCloud(sample)
    pcd_list = pcd_list.vertices

    # for pd in pcd_list:
    #     gm.gen_sphere(pos=pd, radius=0.001, rgba=[1, 0, 0, 1]).attach_to(base)

    #base.run()

    icosphere = gm.gen_sphere(radius=0.5, rgba=[0, 0, 1, 0.1], subdivisions=1)
    sample = icosphere.objtrm.vertices
    # icosphere.attach_to(base)

    d = cm.CollisionModel(mesh)
    com = d.get_com()
    d.set_rgba((0.4, 0.5, 0, 0.4))
    d.attach_to(base)

    base.run()

    name2 = 'CatLying_800_tex'
    mesh2 = tw.TrimeshHu("./kit/", name2 + ".obj")
    e = cm.CollisionModel(mesh2.outputTrimesh)
    e.set_rgba((0.4, 0.5, 0, 1))
    e.set_pos((0, 0, 0.081))
    # e.attach_to(base)
    # base.run()
    # mesh = tw.TrimeshHu("./3dcnnobj/", name + ".obj")

    icosphere = gm.gen_sphere(radius=200, rgba=[0, 0, 1, 0.1], subdivisions=2)
    sample = icosphere.objtrm.vertices
    #icosphere.attach_to(base)

    blocksize = mesh.get_blocksize
    extents = [blocksize * 50] * 3
    # a = rm.homomat_from_posrot(np.array([ 0.00375543,  0.041199  , -0.00062423]))
    a = np.eye(4)
    # gm.gen_frame_box(extents, np.dot(a, mesh.boxhomo)).attach_to(base)
    mesh.meshTransform(rotaxis=np.array([0, 0, 1]), angle=np.radians(0), translation=np.array([0, 0, 0]))
    # mesh.voxelization(0.0015, hollow=True) #"muster"
    # mesh.voxelization(0.001, hollow=True) #"bunnysim04"
    mesh.voxelization(blocksize, hollow=True)  # "mug"
    mesh.get_node_matrix()
    mesh.get_transform()

    # mesh.show_balls()
    # mesh.show_hited_balls(observe_origin=sample[0], target=base, shape = "box", generateSTL=False)

    grasp_info_list = gpa.load_pickle_file(name, './', 'grasp/hande.pickle')
    hnd_rotmat = [grasp_info_list[i][4] for i in range(len(grasp_info_list))]
    gripper_s = rtqhe.RobotiqHE()
    t = rm.rotmat_from_axangle([0, 0, 1], np.deg2rad(90))
    t2 = rm.rotmat_from_axangle([1, 0, 0], np.deg2rad(10))

    t = np.dot(t2, t)

    gripper_s.grip_at_with_jcpose(gl_jaw_center_rotmat=t, jaw_width=0.050, gl_jaw_center_pos=[0, 0.01, 0.002])
    gripper_s.gen_meshmodel().attach_to(base)
    mesh.cpt_briefgrasp(observe_origin=(0, +0.40, 0), target=base, gripper=gripper_s,
                        grasp_info_list=grasp_info_list)
    # mesh.export(this_dir, "box_vox")
    c = cm.CollisionModel(mesh.outputTrimesh)
    # c.set_scale((0.001, 0.001, 0.001))
    c.set_rgba((0, 1, 0, .11))
    # c.attach_to(base)

    base.run()

    objNode = [None]
    voxelNode = [None]
    observeNode = [None]


    def update(textNode, objNode, voxelNode, observeNode, count, task):
        if observeNode[0] is not None:
            observeNode[0].detachNode()
        observeNode[0] = NodePath("observe")
        mesh.show_hited_balls(observe_origin=sample[count[0]], target=observeNode[0])
        gm.gen_sphere(sample[count[0]]).attach_to(observeNode[0])
        observeNode[0].reparent_to(base.render)
        count[0] += 1

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
    taskMgr.doMethodLater(1, update, "addobject", extraArgs=[testNode, objNode, voxelNode, observeNode, count],
                          appendTask=True)

    base.run()