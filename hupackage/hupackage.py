
import modeling.geometric_model as gm
def debugpos(pos, rot, base):
    gm.gen_frame(pos, rot).attach_to(base)