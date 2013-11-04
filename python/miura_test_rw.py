#!/usr/bin/env python
import miura
import ach
import sys
import time
from ctypes import *

m = miura



s = ach.Channel(m.MIURA_CHAN_STATE_NAME)
r = ach.Channel(m.MIURA_CHAN_REF_NAME)
s.flush()
r.flush()
state = m.MIURA_STATE()
ref = m.MIURA_REF()

jnt = 1

flag = 1
while(flag<3):
#    [statusr, framesizer] = r.get(ref, wait=False, last=False)
    [statuss, framesizes] = s.get(state, wait=False, last=False)
 
    if ref.joint[jnt].ref > 0:
        ref.joint[jnt].ref = -0.2
    else:
        ref.joint[jnt].ref = 0.2
    r.put(ref)
    print "jnt: ref = ", state.joint[jnt].ref
    time.sleep(1.0)
r.close()
s.close()

