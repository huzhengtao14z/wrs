import CheckJointsingle as checkangle


import utiltools.robotmath as rm
import time

start = checkangle.getVec()
while True:
    current_vec = checkangle.getVec()
    angle = rm.degree_betweenvector(start, current_vec)
    print(angle)
    time.sleep(1)