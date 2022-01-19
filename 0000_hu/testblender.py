#-*-coding:utf-8-*-
import numpy as np
import trimesh as tri
import basis.trimesh as trimesh
import trimesh.boolean as tb
import trimesh.exchange.stl as tel
# from pandaplotutils import pandactrl as pandactrl
import visualization.panda.world as wd
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.yumi_gripper.yumi_gripper as yg
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as hnde
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode
import basis.robot_math as rm
import modeling.geometric_model as gm
base = wd.World(cam_pos=[0.06, 0.03, 0.09], w=960,
                    h=540, lookat_pos=[0, 0, 0.0])
gm.gen_frame(length=.01, thickness=.0005,).attach_to(base)
# p=[[ 5.00000000e+00,8.00000000e+00, -1.00000000e+01],
#  [ 5.00000000e+00, -8.00000000e+00,  1.00000000e+01],
#  [-2.60197991e-12, -1.00000000e+00, -3.99900278e-07],
#  [-2.60197991e-12,  1.00000000e+00,  3.99900278e-07],
#  [-1.00000000e-07, -1.00000000e+01,  0.00000000e+00],
#  [-1.00000000e-07,  1.00000000e+01,  0.00000000e+00]]
p=[[ 0,0, 0],
 [ 0, .100, 0],
 [0, 0,.100],
   [.100, 0, 0],
 [.100,  .100,  .100],
 [.100, .100,  0],
 [.100,  0, .100],
 [0,  .100, .100]]

m=[[ .050,0, 0],
 [ .050, .150, 0],
 [.050, 0,.150],
   [.150, 0, 0],
 [.150,  .150,  .150],
 [.150, .150,  0],
 [.150,  0, .150],
 [.050,  .150, .150]]
# mesh = trimesh.Trimesh(p).convex_hull
# body=mesh.convex_hull
# b = cm.CollisionModel(mesh)
# b.attach_to(base)

def capsule_link_start_end(start, end, radius = 0.0003):
    start = start
    end = end
    radius = radius
    cylinder = cm.gen_capsule(spos=start, epos=end, radius=radius, section=[5, 5])
    return cylinder

capsule_1 = capsule_link_start_end(start = np.array([0,0,0]), end = np.array([0,0,0.1]))
capsule_2 = capsule_link_start_end(start = np.array([0,0,0]), end = np.array([0,0.2,0]))
a_vertices = capsule_1.objtrm.vertices
a_face_normals = capsule_1.objtrm.face_normals
a_vertex_normals = capsule_1.objtrm.face_normals
a_faces = capsule_1.objtrm.faces

capsule_1.objtrm.export("capsule_1.stl")
capsule_2.objtrm.export("capsule_2.stl")
capsule_3 = tb.union([capsule_1.objtrm, capsule_2.objtrm], engine="blender")
capsule_3.export("capsule_3.stl")
capsule_3_wrsmesh = cm.CollisionModel("capsule_3.stl").attach_to(base)

a_sub = trimesh.Trimesh(p).convex_hull
a_cm = cm.CollisionModel(a_sub)
b_sub = trimesh.Trimesh(m).convex_hull
b_cm = cm.CollisionModel(b_sub)

# a_gm = gm.GeometricModel(a_sub).attach_to(base)
print(a_sub)
a_sub.export("a_sub.stl")
b_sub.export("b_sub.stl")
a_trimesh = tri.load("a_sub.stl")
b_trimesh = tri.load("b_sub.stl")
c = tb.union([a_trimesh, b_trimesh], engine="blender")
c.export("c_sub.stl")
# c_wrsmesh = cm.CollisionModel("c_sub.stl").attach_to(base)
# c = gm.GeometricModel(c_wrsmesh).attach_to(base)
# tel.export_stl_ascii(a_mesh)
# a_mesh.export('a_mesh_1.stl')
# print("a", a)
# print("a_mesh", a_mesh)
# b_mesh = b.objtrm
# b_mesh.export('b_mesh.stl')
# a_trimesh = tri.load("a_mesh.stl")
# b_trimesh = tri.load("b_mesh.stl")
# c_mesh = tb.union([a_trimesh, b_trimesh], engine = "blender")
# c = gm.GeometricModel(c_mesh)
# print("c", c)
# c = cm.CollisionModel(c_mesh)
# c = gm.GeometricModel(c_mesh)
# c.attach_to(base)
base.run()
m=[[ .050,0, 0],
 [ .050, .100, 0],
 [.050, 0,.100],
   [.150, 0, 0],
 [.150,  .100,  .100],
 [.150, .100,  0],
 [.150,  0, .100],
 [.050,  .100, .100]]
# base.pggen.plotAxis(base.render)
# print(Vec3(100, 100, 100), type(Vec3(100, 100, 100)))
b.set_rgba((0, 1, 0, 1))
#
# b.setPos(Vec3(100, 100, 100))
b.attach_to(base)
mesh_b = trimesh.Trimesh(m)
body_m=mesh_b.convex_hull
bm = cm.CollisionModel(body_m)
bm.attach_to(base.render)
a = tb.union([mesh, mesh_b], engine = "blender")
am= gm.GeometricModel(a)
am.attach_to(base)
# taskMgr.doMethodLater(0.5, update, 'update', extraArgs=[b,counter], appendTask=True)
base.run()