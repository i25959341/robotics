import brickpi
import time
import random
import time
import sys
import random
import numpy as np
from robot_controller import *

from particleDataStructures import *

can = Canvas()
mymap = Map(can);
particle = Particles(can)

mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h

mymap.add_wall((84,30,180,30));
mymap.add_wall((180,30,180,54));
mymap.add_wall((180,54,138,54));
mymap.add_wall((138,54,138,168));
mymap.add_wall((138,168,114,168));
mymap.add_wall((114,168,114,84));
mymap.add_wall((114,84,84,84));
mymap.add_wall((84,84,84,30));

mymap.draw();

controller = RobotController(particle)

particleSet = controller.initParticles()

#controller.go2(82)
#controller.turnSonar90()
self.turnSonar(-90)

time.sleep(3)
controller.turnSonar90Neg()
#controller.go2(82)
controller.turnSonar90Neg()
controller.turnSonar90Neg()
controller.turnSonar90Neg()
