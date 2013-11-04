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
print 'Miura server running'
while(True):
    [statusr, framesizer] = r.get(ref, wait=True, last=False)
#    [statuss, framesizes] = s.get(state, wait=False, last=False)
    state.joint[jnt].ref = ref.joint[jnt].ref
    s.put(state)
r.close()
s.close()

