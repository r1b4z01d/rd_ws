#!/usr/bin/env python3
import time 
import pyDHgripper
import pyDHgripper.pyDHgripper
gripper = pyDHgripper.pyDHgripper.PGE(port='/tmp/ttyUR')

gripper.init_state()
gripper.init_feedback()
time.sleep(1)
print(gripper.read_state())

i = 0
while i <= 100:
    gripper.set_force(val=100)
    gripper.set_force(val=100)
    gripper.set_pos(val=0)
    print(gripper.read_pos())
    time.sleep(0.25)
    gripper.set_pos(val=1000)
    print(gripper.read_pos())
    time.sleep(0.25)
    i += 1

