from panda3d.bullet import BulletRigidBodyNode
# from panda3d.bullet.BulletRigidBodyNode import setRestitution
import direct.directbase.DirectStart
from panda3d.core import Vec3
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletCharacterControllerNode
from panda3d.bullet import BulletCapsuleShape
from panda3d.bullet import ZUp

base.cam.setPos(0, -10, 0)
base.cam.lookAt(0, 0, 0)

# World
world = BulletWorld()
world.setGravity(Vec3(0, 0, -9.81))

height = 1.75
radius = 0.4
shape = BulletCapsuleShape(radius, height - 2 * radius, ZUp)

playerNode = BulletCharacterControllerNode(shape, 0.4, 'Player')
playerNP = world.render.attachNewNode(playerNode)  # This is where self.worldNP is being mentioned
playerNP.setPos(-2, 0, 14)
playerNP.setH(45)
playerNP.setCollideMask(BitMask32.allOn())

world.attachCharacter(playerNP.node())


def update(task):
    dt = globalClock.getDt()
    world.doPhysics(dt)
    world.doPhysics(dt)
    return task.cont


taskMgr.add(update, 'update')
base.run()