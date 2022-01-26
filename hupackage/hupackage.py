import basis.robot_math as rm
import modeling.geometric_model as gm
import numpy as np
def debugpos(pos, rot, base):
    gm.gen_frame(pos, rot).attach_to(base)


def normal_from_3point(p1, p2, p3):
    x1, y1, z1 = p1[0], p1[1], p1[2]
    x2, y2, z2 = p2[0], p2[1], p2[2]
    x3, y3, z3 = p3[0], p3[1], p3[2]
    # print(x3, y3, z3)
    a = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1)
    b = (z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1)
    c = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)
    return rm.unit_vector(np.array([a, b, c]))