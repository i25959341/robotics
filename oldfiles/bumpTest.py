from bot_utilitySensor import *

import brickpi
import time

interface=brickpi.Interface()
motors = [2, 3]
touchPort = [2,3]

initInterface(interface, motors, touchPort)

goStraightBump(interface, motors, touchPort)

interface.terminate()
