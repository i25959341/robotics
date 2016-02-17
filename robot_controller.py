import brickpi
import time
import random
import math

class RobotController:
    def __init__(self):
        self.motors = [2,3]
        self.port = 0
        self.initInterface()
        self.initWall()
        self.pi =3.14159265359

    def initInterface(self):
        self.interface = brickpi.Interface()
        self.interface.initialize()

        self.interface.motorEnable(self.motors[0])
        self.interface.motorEnable(self.motors[1])

        motorParams.maxRotationSpeed = 13.0
        motorParams.feedForwardGain = 255/20.0
        motorParams.minPWM = 18.0
        motorParams.pidParameters.minOutput = -255.0
        motorParams.pidParameters.maxOutput = 255.0
        motorParams.pidParameters.k_p = 250.0
        motorParams = self.interface.MotorAngleControllerParameters()
        motorParams.maxRotationAcceleration = 8.0
        motorParams.pidParameters.k_i = 809.0909
        motorParams.pidParameters.k_d = 11.0

        motorParams2 = self.interface.MotorAngleControllerParameters()
        motorParams2.maxRotationAcceleration = 8.0
        motorParams2.maxRotationSpeed = 13.0
        motorParams2.feedForwardGain = 255/20.0
        motorParams2.minPWM = 18.0
        motorParams2.pidParameters.minOutput = -255.0
        motorParams2.pidParameters.maxOutput = 255.0
        motorParams2.pidParameters.k_p = 250.0
        motorParams2.pidParameters.k_i = 852.1739
        motorParams2.pidParameters.k_d = 11.5

        self.interface.setMotorAngleControllerParameters(self.motors[0], motorParams)
        self.interface.setMotorAngleControllerParameters(self.motors[1], motorParams2)
        
        self.interface.sensorEnable(self.port, brickpi.SensorType.SENSOR_ULTRASONIC)
        
    def initWall(self):
        self.wall = []
        wall.append((0,0,0,168))
        wall.append((0,168,84,168))
        wall.append((84,126,84,210))
        wall.append((84,210,168,210))
        wall.append((168,210,168,84))
        wall.append((168,84,210,84))
        wall.append((210,84,210,0))
        wall.append((210,0,0,0))

    def goStraight(self, distCm):
        radianPerCm = 0.3725
        angle = distCm * radianPerCm
        self.interface.increaseMotorAngleReferences(self.motors, [angle, angle])

        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.1)
            ## Maybe add a break after x seconds

    def turn(self, angleDeg):
        radianPerDegre = 0.06405
        angle = angleDeg * radianPerDegre
        self.interface.increaseMotorAngleReferences(self.motors, [angle, -angle])

        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.1)

    def initParticles(self, NUMBER_OF_PARTICLES):
        particleSet=[]
        weight = 1/NUMBER_OF_PARTICLES
        tuple =(0,0,0)
        for i in range(NUMBER_OF_PARTICLES):
            particleSet.append(tuple)

        return particleSet

    def go10Cm(self, particleSet):
        ePer10cm = 0.2
        fPer10cm = 0.2
        D = 10
        newparticleSet = []

        self.goStraight(D)
        for particle in particleSet:
            error = random.gauss(0, ePer10cm)
            x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
            y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
            theta = (particle[2]+random.gauss(0,fPer10cm))%360
            particle = (x, y, theta)
            newparticleSet.append(particle)

        return (newparticleSet)

    def turn90Deg(self, particleSet):
        gPer90 = 1
        newparticleSet = []
        angle = 90

        self.turn(angle)
        for particle in particleSet:
            new_angle = (particle[2] + angle + random.gauss(0,gPer90))%360
            new_particle = (particle[0], particle[1], new_angle)
            newparticleSet.append(new_particle)

        return (newparticleSet)

    def scale(self, particleSet):
        newparticleSet=[]
        offsetx=200
        offsety=600
        scale=10

        for particle in particleSet:
            newx = particle[0]*scale+offsetx
            newy = -particle[1]*scale+offsety
            newtheta = particle[2]
            new_particle = ( newx , newy , newtheta)
            newparticleSet.append(new_particle)
        return (newparticleSet)

    def position(self, particleSet, NUMBER_OF_PARTICLES):
        sumX = 0
        sumY = 0
        sumTheta = 0

        for particle in particleSet:
            sumX=sumX+particle[0]
            sumY=sumY+particle[1]
            sumTheta=sumTheta+particle[2]

        sumX=sumX/NUMBER_OF_PARTICLES
        sumY=sumY/NUMBER_OF_PARTICLES
        sumTheta=(sumTheta/NUMBER_OF_PARTICLES)%360

        return (sumX, sumY, sumTheta)

    def go(self, distCm, particleSet):
        ePer10cmSqt=0.01
        fPer10cmSqt=0.01

        ePer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)
        fPer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)

        D=distCm
        newparticleSet=[]

        self.goStraight(D)

        for particle in particleSet:
            error = random.gauss(0,ePer10cm)
            x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
            y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
            theta = (particle[2]+random.gauss(0,fPer10cm))%360
            particle = (x, y, theta)
            newparticleSet.append(particle)

        return (newparticleSet)

    def rotate(self, angleDeg, particleSet):
        gPer90Sqt = 0.1

        gPer90 = math.sqrt(gPer90Sqt*abs(angleDeg)/90)
        newparticleSet = []
        angle = angleDeg

        self.turn(angle)
        for particle in particleSet:
            new_angle = (particle[2] + angle + random.gauss(0,gPer90))%360
            new_particle = (particle[0], particle[1], new_angle)
            newparticleSet.append(new_particle)

        return (newparticleSet)

    def navigate(self, particleSet):
        currentX = 0
        currentY = 0
        currentTheta = 0

        pi = 3.14159265359

        print "Ctr-C to cancel"

        while True:
            destX=input('Please Enter the X co-ordinate  ')
            destY=input('Please Enter the Y co-ordinate  ')

            dx = destX - currentX
            dy = destY - currentY

            D = math.sqrt(dx*dx+dy+dy)
            alpha = math.atan2(dy,dx)

            beta = alpha - currentTheta/360*2*pi

            if beta > pi:
                beta = beta - 2*pi
            elif beta<=-pi:
                beta = beta + 2*pi

            newparticleSet = self.rotate(360*beta/(2*pi), particleSet)
            newparticleSet2 = self.go(D, newparticleSet)

            currentposition = self.position(newparticleSet2, 100)

            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]
            
    def calculate_likelihood(x, y, theta, z):
        #change theta
        theta = theta/360 * self.pi
        
        st_dev = 0.9
        K = 0.001
        maxAngle= 0.610865
        
        hit_m = 100000
        
        for wall in walls: 
            (ax, ay, bx, by) = wall
            m = ((by - ay) * (ax - x) - (bx - ax) * (ay - y)) / ((by - ay) * math.cos(theta) - (bx - ax) * math.sin(theta))
        
            hitting_point = (x + m * math.cos(theta), y + m * math.sin(theta))
            
            #calculate angle
            dis = math.sqrt((ay-by)**2+(bx-ax)**2) 
            beta = math.acos(((math.cos(theta)*(ay-by)+math.sin(theta)*(bx-ax))/(dis))
            
            #check if it hits the wall
            if (min(ax, bx) <= hitting_point[0] <= max(ax, bx) and min(ay, by) <= hitting_point[1] <= max(ay, by)):
                if beta < maxAngle:
                    hit_m = min(hit_m, m)
                
        p = exp(-((z - m)**2) / (2 * st_dev**2)) + K
        
        return p
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
