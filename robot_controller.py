import brickpi
import time
import random
import math


class RobotController:
    def __init__(self, _particle):
        self.motors = [2,3]
        self.port = 0
        self.initInterface()
        self.initWall()
        self.pi =3.14159265359
        self.particleSet=[]
        self.NUMBER_OF_PARTICLES=100
        self.initParticles()
        self.radianPerCm = 0.3725
        self.radianPerDegre = 0.062985
        self.particle = _particle

    def initInterface(self):
        self.interface = brickpi.Interface()
        self.interface.initialize()

        self.interface.motorEnable(self.motors[0])
        self.interface.motorEnable(self.motors[1])

        motorParams = self.interface.MotorAngleControllerParameters()
        motorParams.maxRotationAcceleration = 5.0
        motorParams.maxRotationSpeed = 9.0
        motorParams.feedForwardGain = 255/20.0
        motorParams.minPWM = 18.0
        motorParams.pidParameters.minOutput = -255.0
        motorParams.pidParameters.maxOutput = 255.0
        motorParams.pidParameters.k_p = 250.0
        motorParams.pidParameters.k_i = 1009.0909
        motorParams.pidParameters.k_d = 11.0

        motorParams2 = self.interface.MotorAngleControllerParameters()
        motorParams2.maxRotationAcceleration = 5.0
        motorParams2.maxRotationSpeed = 9.0
        motorParams2.feedForwardGain = 255/20.0
        motorParams2.minPWM = 18.0
        motorParams2.pidParameters.minOutput = -255.0
        motorParams2.pidParameters.maxOutput = 255.0
        motorParams2.pidParameters.k_p = 250.0
        motorParams2.pidParameters.k_i = 1052.1739
        motorParams2.pidParameters.k_d = 11.5

        self.interface.setMotorAngleControllerParameters(self.motors[0], motorParams)
        self.interface.setMotorAngleControllerParameters(self.motors[1], motorParams2)

        self.interface.sensorEnable(self.port, brickpi.SensorType.SENSOR_ULTRASONIC)

    def initWall(self):
        self.walls = []
        self.walls.append((0,0,0,168))
        self.walls.append((0,168,84,168))
        self.walls.append((84,126,84,210))
        self.walls.append((84,210,168,210))
        self.walls.append((168,210,168,84))
        self.walls.append((168,84,210,84))
        self.walls.append((210,84,210,0))
        self.walls.append((210,0,0,0))

    def goStraight(self, distCm):
        #radianPerCm = 0.3725
        angle = distCm * self.radianPerCm
        self.interface.increaseMotorAngleReferences(self.motors, [angle, angle])

        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.1)
            ## Maybe add a break after x seconds

    def turn(self, angleDeg):
        #radianPerDegre = 0.06405
        angle = angleDeg * self.radianPerDegre
        self.interface.increaseMotorAngleReferences(self.motors, [angle, -angle])

        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.1)

    def initParticles(self):
        self.particleSet = []
        weight = 1.0/self.NUMBER_OF_PARTICLES
        tuple =(84,30,0,weight)
        for i in range(self.NUMBER_OF_PARTICLES):
            self.particleSet.append(tuple)



    def go10Cm(self):
        ePer10cm = 0.2
        fPer10cm = 0.2
        D = 10
        newparticleSet = []

        self.goStraight(D)

        for particle in self.particleSet:
            error = random.gauss(0, ePer10cm)
            x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
            y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
            theta = (particle[2]+random.gauss(0,fPer10cm))%360
            particle = (x, y, theta,particle[3])
            newparticleSet.append(particle)

        self.particleSet = newparticleSet

    def turn90Deg(self):
        gPer90 = 1
        newparticleSet = []
        angle = 90

        self.turn(angle)
        for particle in self.particleSet:
            new_angle = (particle[2] + angle + random.gauss(0,gPer90))%360
            new_particle = (particle[0], particle[1], new_angle ,particle[3])
            newparticleSet.append(new_particle)

        self.particleSet = (newparticleSet)

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

    def position(self):
        sumX = 0
        sumY = 0
        sumTheta = 0

        for particle in self.particleSet:
            sumX=sumX+particle[0]
            sumY=sumY+particle[1]
            sumTheta=sumTheta+particle[2]

        sumX=sumX/self.NUMBER_OF_PARTICLES
        sumY=sumY/self.NUMBER_OF_PARTICLES
        sumTheta=(sumTheta/self.NUMBER_OF_PARTICLES)%360

        return (sumX, sumY, sumTheta)

    def position2(self):
        sumX = 0
        sumY = 0
        sumTheta = 0
        weight=0

        for particle in self.particleSet:
            sumX=sumX+particle[0]*particle[3]
            sumY=sumY+particle[1]*particle[3]
            sumTheta=sumTheta+particle[2]*particle[3]
            weight = weight +particle[3]

        return (sumX/weight, sumY/weight, sumTheta/weight)


    def go(self, distCm):
        ePer10cmSqt=2
        fPer10cmSqt=2

        ePer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)
        fPer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)

        D=distCm
        newparticleSet=[]

        self.goStraight(D)

        for particle in self.particleSet:
            error = random.gauss(0,ePer10cm)
            x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
            y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
            theta = (particle[2]+random.gauss(0,fPer10cm))%360
            particle = (x, y, theta, particle[3])
            newparticleSet.append(particle)

        self.particleSet = (newparticleSet)

    def rotate(self, angleDeg):
        gPer90Sqt = 0.1

        gPer90 = math.sqrt(gPer90Sqt*abs(angleDeg)/90)
        newparticleSet = []
        angle = angleDeg

        self.turn(angle)
        for particle in self.particleSet:
            new_angle = (particle[2] + angle + random.gauss(0,gPer90))%360
            new_particle = (particle[0], particle[1], new_angle, particle[3])
            newparticleSet.append(new_particle)

        self.particleSet =  (newparticleSet)

    def navigate(self):
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

            self.rotate(360*beta/(2*pi))
            self.go(D)

            currentposition = self.position(self.particleSet)

            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]

    def calculate_likelihood(self,x, y, theta, z):
        #change theta
        theta = theta/360 * self.pi

        st_dev = 5.0
        K = 0.001    # fat tails
        maxAngle= 0.610865

# angle after which the sonar's measurements arent reliable anymore

        hit_m = 100000

        c=0

        # Finding the closest wall in the direction
        for wall in self.walls:
            (ax, ay, bx, by) = wall

            divisor = (((by - ay) * math.cos(theta) - (bx - ax) * math.sin(theta)))

            if divisor==0:
                continue

            m = ((by - ay) * (ax - x) - (bx - ax) * (ay - y)) / divisor

            if m <0:
                continue

            hitting_point = (x + m * math.cos(theta), y + m * math.sin(theta))

            #calculate angle
            dis = math.sqrt((ay-by)**2+(bx-ax)**2)   # length of hte wall
            beta = math.acos(((math.cos(theta)*(ay-by)+math.sin(theta)*(bx-ax))/dis))

            #check if it hits the wall
            if (((min(ax, bx) <= hitting_point[0]<= max(ax, bx))) and  ( min(ay, by) <= hitting_point[1] <= max(ay, by)) ):
                if (abs(beta) < maxAngle):
                    hit_m = min(hit_m, m)
                    c=c+1

        #if c==0:
            #return 1

        # Computing the likelihood
        p = math.exp(-((z - hit_m)**2) / (2 * (st_dev**2))) + K

        print hit_m

        return p


    def getSonarReading(self):
        usReading=[]
        for i in range(20):
            valSonar = self.interface.getSensorValue(self.port)[0]
            if(valSonar < 255):
                usReading.append(valSonar)
            if not usReading :
                print "Failed US reading"
        nbMeasures = len(usReading)
        if (nbMeasures == 0):
            print 255
            return 255
        else:
            index = int(nbMeasures/2)
            usReading.sort()
            print 'z:' + str(usReading[index] + 16.3)
            return usReading[index]


    def updateParticles(self):
        z = self.getSonarReading()
        t = 16.3
        z = z+t

        newParticles=[]

        print "old" + str(self.particleSet)

        for particle in self.particleSet:
            (x,y,theta,w)= particle
            likelihood = self.calculate_likelihood(x, y, theta, z)
            newParticles.append((x,y,theta,w*likelihood))

        print "new" + str(newParticles)


        self.particleSet=newParticles
        self.normalise()
        self.resampling()
        
        print "after" + str(self.particleSet)



    def normalise(self):
        sumOfWeight=0.0
        newParticles=[]

        for particle in self.particleSet:
            sumOfWeight=sumOfWeight+particle[3]


        for particle in self.particleSet:
            (x,y,z,w) = particle
            newParticles.append((x,y,z,1.0*w/sumOfWeight))

        self.particleSet=newParticles




    def resampling(self):
        cumulative=[self.particleSet[0]]
        newParticleSet=[]

        for particle in self.particleSet[1:]:
            (x,y,theta,w)=particle
            cumulative.append((x,y,theta,w+cumulative[-1][3]))

        for particle1 in self.particleSet:
            i=random.random()
            for particle2 in cumulative:
                if(i<=particle2[3]):
                    (x,y,theta,w)=particle2
                    newParticleSet.append((x,y,theta,1.0/self.NUMBER_OF_PARTICLES))
                    break

        self.particleSet=newParticleSet




    def go2(self, distCm):
        ePer10cmSqt=2
        fPer10cmSqt=2

        ePer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)
        fPer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)

        #D=distCm/4
        D = distCm
        newparticleSet=[]

        #for i in range(4):
        self.goStraight(D)
        for particle in self.particleSet:
            error = random.gauss(0,ePer10cm)
            x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
            y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
            theta = (particle[2]+random.gauss(0,fPer10cm))%360
            particle = (x, y, theta, particle[3])
            newparticleSet.append(particle)

        self.particleSet = (newparticleSet)

        self.updateParticles()

    def rotate2(self, angleDeg):
        gPer90Sqt = 2

        gPer90 = math.sqrt(gPer90Sqt*abs(angleDeg)/90)
        newparticleSet = []
        angle = angleDeg


        self.turn(angle)
        for particle in self.particleSet:
            new_angle = (particle[2] + angle + random.gauss(0,gPer90))%360
            new_particle = (particle[0], particle[1], new_angle, particle[3])
            newparticleSet.append(new_particle)

        self.particleSet =  (newparticleSet)
        self.updateParticles()


    def distance_to_target(self, destX, destY, currentX, currentY):
        dx = destX - currentX
        dy = destY - currentY

        return math.sqrt(dx*dx+dy*dy)


    def navigate2(self):
        points = [
            (180, 30),
            (180, 54),
            (138, 54),
            (138, 168),
            (114, 168),
            (114, 84),
            (84, 84),
            (84, 30)
        ]

        currentX = 84
        currentY = 30
        currentTheta = 0

        pi = 3.14159265359

        print "Ctr-C to cancel"

        for (destX, destY) in points:

            dx = destX - currentX
            dy = destY - currentY

            alpha = math.atan2(dy,dx)

            beta = alpha - currentTheta/360*2*pi

            if beta > pi:
                beta = beta - 2*pi
            elif beta < -pi:
                beta = beta + 2*pi

            self.rotate2(360*beta/(2*pi))
            self.particle.update(self.particleSet)
            self.particle.draw()

            D = self.distance_to_target(destX, destY, currentX, currentY)
            while D > 0:
                #print 'D' + str(D)
                self.go2(min(D, 15))
                self.particle.update(self.particleSet)
                self.particle.draw()
                if (D < 5):
                    break
                currentposition = self.position2()
                D = self.distance_to_target(destX, destY, currentposition[0], currentposition[1])


            currentposition = self.position2()


            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]
