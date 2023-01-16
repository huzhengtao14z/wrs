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
import os
import pickle
import basis.data_adapter as da
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
    from shapely.geometry import Polygon, MultiPolygon
    base = wd.World(cam_pos=[2.01557, 0.637317, 1.88133], w=960,
                    h=540, lookat_pos=[0, 0, 0])
    # gm.gen_frame().attach_to(base)
    this_dir, this_filename = os.path.split(__file__)
    adress = "kit_model/"
    files = os.listdir(adress)[62:]
    comdict = {}

    for names in files:
        name = names
        print(name)
        obj = tw.TrimeshHu(meshpath=adress, name=name, scale=0.001)
        mesh = obj.outputTrimesh
        #####重心
        d = cm.CollisionModel(mesh)
        d = d.get_com()
        testmesh = gm.GeometricModel(mesh)
        testmesh.set_rgba([1, 0, 0, 1])



        icosphere = gm.gen_sphere(radius=0.15, rgba=[0, 0, 1, 0.1], subdivisions=0)
        sample = icosphere.objtrm.vertices[10:]
        # icosphere.attach_to(base)
        # profile[name].append(sample)
        # profile[name].append(d)
        count = 0
        for origin in sample:
            gm.gen_sphere(origin).attach_to(base)
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
            viewed = intersector.intersects_first(ray_origins=origin_list, ray_directions=check_list)
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

            # surface = hm.getsurfaceequation(normal = np.array(np.array([0,0,0])-origin), pnt = np.array([0,0,0]))
            projected_vertices = []
            for item in viewed_vertices:
                projected_point = rm.project_to_plane(item, np.array([0,0,0]), rm.unit_vector(origin))
                projected_vertices.append(projected_point)
                # gm.gen_sphere(projected_point, 0.001).attach_to(base)0
                # gm.gen_sphere(item, 0.001).attach_to(base)
            polygon_vertices = []

            projected_mesh = trimesh.Trimesh(vertices=projected_vertices, faces=viewed_faces)
            projected_mesh.export("testprojected.stl")

            # projected_mesh = trimeshWan.Trimesh(vertices=projected_vertices, faces=viewed_faces)
            projectedmodel = gm.GeometricModel("testprojected.stl")
            projectedmodel.set_rgba([1,0,0,0.3])
            projectedmodel.attach_to(base)
            plane_rot = np.linalg.inv(rm.rotmat_between_vectors(np.array([0,0,1]),  rm.unit_vector(origin)))

            T_point = rm.homomat_from_posrot(rot = plane_rot)

            for face in viewed_faces:
                polygon_vertices.append(Polygon([projected_vertices[face[0]][:-1], projected_vertices[face[1]][:-1], projected_vertices[face[2]][:-1]]))
                a = rm.homomat_transform_points(T_point, np.asarray([projected_vertices[face[0]], projected_vertices[face[1]],projected_vertices[face[2]]]))
                # gm.gen_sphere(a[0], radius=0.001, rgba = [1,0,0,1]).attach_to(base)
                # gm.gen_sphere(a[1], radius=0.001, rgba=[1, 0, 0, 1]).attach_to(base)
                # gm.gen_sphere(a[2], radius=0.001, rgba=[1, 0, 0, 1]).attach_to(base)
                polygon_vertices.append(Polygon(list(a)))
            projectedpic = MultiPolygon(polygon_vertices)
            cccc = projectedpic.centroid
            print(cccc)

            ccccc = np.array([cccc.x,cccc.y,0])
            # ccccc = rm.homomat_transform_points(rm.homomat_from_posrot(rot = plane_rot), ccccc)


            # projectedmodel.set_rotmat(plane_rot)



            # print(projectedmodel.objtrm.centroid)
            # test.attach_to(base)


            viewedmesh=trimesh.Trimesh(vertices = viewed_vertices, faces=viewed_faces)
            viewedmesh.export("testtest.stl")
            test=gm.GeometricModel("testtest.stl")
            test.set_rgba([1,0,0,1])
            # test.attach_to(base)
            Mesh = o3d.io.read_triangle_mesh("testtest.stl")
            Mesh.compute_vertex_normals()
            pcd1 = Mesh.sample_points_poisson_disk(number_of_points = 500)
            pcd1_np=vdda.o3dpcd_to_parray(pcd1)

            # r=uniform_random_rotate()
            # print(r)
            # pcd1_rot_np = np.dot(pcd1_np, r)
            # pcd1_rot=copy.deepcopy(pcd1)
            # pcd1_rot.rotate(r, center=(0,0,0))
            # o3d.visualization.draw_geometries([pcd1, pcd1_rot])

            # pcd1_rot_np = vdda.o3dpcd_to_parray(pcd1_rot)
            # pcd1_rot_np_v = gm.GeometricModel(pcd1_rot_np).attach_to(base)
            # print(pcd1_np.shape)
            # print(pcd1_np)

            # print(pcd1_rot_np)
            center=np.average(pcd1_np,axis = 0)
            tm = np.linalg.inv(rm.homomat_from_posrot(center))
            pcd1.transform(tm)
            pcd1_np=vdda.o3dpcd_to_parray(pcd1)
            com = rm.homomat_transform_points(tm, d)
            commodel0 = gm.gen_sphere(com, rgba=(0, 1, 0, 1), radius=0.002).attach_to(base)

            gm.gen_cylinder(homomat = rm.homomat_from_posrot(com, plane_rot)).attach_to(base)

            com = rm.homomat_transform_points(rm.homomat_from_posrot(rot = plane_rot), com)
            # commodel = gm.gen_sphere(com,rgba=(0,1,0,1), radius=0.002)
            # commodel.attach_to(base)
            pcd1_v = gm.GeometricModel(pcd1_np).attach_to(base)
            testmesh.set_homomat(tm)
            testmesh.set_rgba((0,0,1,0.2))
            # testmesh.attach_to(base)

            # gm.gen_sphere(ccccc, radius=0.002, rgba=(1, 1, 0, 1)).attach_to(base)  # 平面上的
            gm.gen_sphere(rm.homomat_transform_points(rm.homomat_inverse(T_point),ccccc), radius=0.002, rgba=(1, 1, 0, 1)).attach_to(base)  # 转回原投影位

            # gm.gen_sphere(com).attach_to(base)
            # pcd1_np_v = gm.GeometricModel(pcd1_rot_np).attach_to(base)
            base.run()
            # mu, sigma = 0, 0.001  # mean and standard deviation
            #
            # source_noisy = apply_noise(pcd1, mu, sigma)


            objname = name
            objname = objname.replace('.stl', '')
            filename = objname + '-' + str(count) +".ply"
            comname = objname + '-' + str(count)
            comdict[comname] = com
            # filename2 = filename = objname + str(count) +".stl"
            #filename3 = "noizy" + objname + str(count) + ".ply"
            # adress_pc = ""
            #adress_pc = "pirtial_pc/"+filename
            # adress_mesh=""
            # adress_mesh= "partial_mesh/" + filename2
            adress_pc = "cc_pcd/" + filename

            o3d.io.write_point_cloud(adress_pc, pcd1)
            #viewedmesh.export(adress_pc)
            #ptCloud = o3d.io.read_point_cloud(adress_pc)
            #print(ptCloud)
            count += 1


    with open('data/kit/pairtial/cc_com.pickle', 'wb') as file:
        pickle.dump(comdict, file)
    print('finish')
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