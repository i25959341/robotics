import brickpi
import time
import random
import time
import sys
import random
import numpy as np
from robot_controller import RobotController

controller = RobotController()

particleSet = controller.initParticles()


controller.navigate2()
