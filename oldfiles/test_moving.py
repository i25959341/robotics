import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [2,3]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 8.0
motorParams.maxRotationSpeed = 16.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 240.0
motorParams.pidParameters.k_i = 1309.0909
motorParams.pidParameters.k_d = 11.0

motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration = 8.0
motorParams2.maxRotationSpeed = 16.0
motorParams2.feedForwardGain = 255/20.0
motorParams2.minPWM = 18.0
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = 240.0
motorParams2.pidParameters.k_i = 1252.173913
motorParams2.pidParameters.k_d = 11.5

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams2)

brickpi.Interface.startLogging(interface,"test.txt")

while True:
	angle = float(input("Enter a angle to rotate (in radians): "))

	interface.increaseMotorAngleReferences(motors,[angle,-angle])

	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]

		time.sleep(0.1)

	print "Destination reached!"
	brickpi.Interface.stopLogging(interface)
	

interface.terminate()
