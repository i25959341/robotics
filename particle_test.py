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

#for i in range(20):
#    particleSet = go10Cm(interface, motors,particleSet)
#    print "drawParticles:" + str(particleSet)
#    print particleSet

#goStraight(10, interface, motors)
#for particle in particleSet:
#    print "drawParticles:" + str(particle)

print "drawParticles:" + str(particleSet)

for i in range(5):
    particleSet = go10Cm(interface, motors, particleSet)
    print "drawParticles:" + str(particleSet)
    particleSet = turn90Deg(interface, motors, particleSet)
    print "drawParticles:" + str(particleSet)
