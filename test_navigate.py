import brickpi
import time
import random
import time
import sys
import random
import numpy as np
from bot_utilities import *

interface=brickpi.Interface()
interface.initialize()

motors = [2,3]

initInterface(interface, motors)

particleSet=initParticles(100)





'''
line1 = (200, 200, 200, 600) 
line2 = (200, 200, 600, 200)
line3 = (600, 600, 600, 200)
line4 = (600, 600, 200, 600)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
print "drawLine:" + str(line3)
print "drawLine:" + str(line4)

newparticleSet = scale(particleSet)

print "drawParticles:" + str(newparticleSet)


navigate(interface, motors,particleSet)
'''