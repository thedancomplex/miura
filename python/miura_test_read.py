#!/usr/bin/env python
import miura
import ach
import sys
import time
from ctypes import *

m = miura



s = ach.Channel(ha.MIURA_CHAN_STATE_NAME)
r = ach.Channel(ha.MIURA_CHAN_REF_NAME)
s.flush()
r.flush()
state = ha.HUBO_STATE()
ref = ha.HUBO_REF()


flag = 1
while(flag<3):
#    [statusr, framesizer] = r.get(ref, wait=False, last=False)
    [statuss, framesizes] = s.get(state, wait=False, last=False)
    if ref.ref[ha.WST] > 0:
        ref.ref[ha.WST] = -0.2
    else:
        ref.ref[ha.WST] = 0.2
    r.put(ref)
    print "WST: ref = ", state.joint[ha.WST].ref
    time.sleep(1.0)
r.close()
s.close()

