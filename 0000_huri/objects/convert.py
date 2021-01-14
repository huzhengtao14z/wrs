import math
import numpy as np
import basis.robotmath as rm
import modeling.meshtools as mt

if __name__ == '__main__':
    '''
    author: weiwei
    date: 20201207osaka
    '''
    mt.convert_to_stl("tubebig.stl", "tubebig.stl", scale_ratio=.001)