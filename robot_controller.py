import brickpi
import time
import random
import math


class RobotController:
    def __init__(self, _particle):
        self.motor3 = 1
        self.motors = [2,3]
        self.port = 2
        self.initInterface()
        self.initWall()
        self.pi =3.14159265359
        self.particleSet=[]
        self.NUMBER_OF_PARTICLES=100

        self.radianPerCm = 0.2975
        self.radianPerDegre = 0.0469
        self.radianPerDegreNeg = 0.0469
        self.particle = _particle

        self.initParticles()

    def initInterface(self):
        self.interface = brickpi.Interface()
        self.interface.initialize()

        self.interface.motorEnable(self.motors[0])
        self.interface.motorEnable(self.motors[1])

        motorParams = self.interface.MotorAngleControllerParameters()
        motorParams.maxRotationAcceleration = 6.0
        motorParams.maxRotationSpeed = 13.0
        motorParams.feedForwardGain = 255/20.0
        motorParams.minPWM = 18.0
        motorParams.pidParameters.minOutput = -255.0
        motorParams.pidParameters.maxOutput = 255.0
        motorParams.pidParameters.k_p = 180.0
        motorParams.pidParameters.k_i = 457.1428571
        motorParams.pidParameters.k_d = 8.875

        motorParams2 = self.interface.MotorAngleControllerParameters()
        motorParams2.maxRotationAcceleration = 6.0
        motorParams2.maxRotationSpeed = 13.0
        motorParams2.feedForwardGain = 255/20.0
        motorParams2.minPWM = 18.0
        motorParams2.pidParameters.minOutput = -255.0
        motorParams2.pidParameters.maxOutput = 255.0
        motorParams2.pidParameters.k_p = 180.0
        motorParams2.pidParameters.k_i = 457.1428571
        motorParams2.pidParameters.k_d = 7.875

        self.interface.setMotorAngleControllerParameters(self.motors[0], motorParams)
        self.interface.setMotorAngleControllerParameters(self.motors[1], motorParams2)

        self.interface.sensorEnable(self.port, brickpi.SensorType.SENSOR_ULTRASONIC)


        self.interface.motorEnable(self.motor3)

        motorParams3 = self.interface.MotorAngleControllerParameters()
        motorParams3.maxRotationAcceleration = 5.0
        motorParams3.maxRotationSpeed = 9.0
        motorParams3.feedForwardGain = 255/20.0
        motorParams3.minPWM = 18.0
        motorParams3.pidParameters.minOutput = -255.0
        motorParams3.pidParameters.maxOutput = 255.0
        motorParams3.pidParameters.k_p = 250.0
        motorParams3.pidParameters.k_i = 1052.1739
        motorParams3.pidParameters.k_d = 11.5

        self.interface.setMotorAngleControllerParameters(self.motor3, motorParams3)


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
        angle = -1.0* distCm * self.radianPerCm
        self.interface.increaseMotorAngleReferences(self.motors, [0.97*angle, angle])

        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.1)
            ## Maybe add a break after x seconds

    def turn(self, angleDeg):
        if angleDeg>0:
            angle = angleDeg * self.radianPerDegre
        else:
            angle = angleDeg * self.radianPerDegreNeg

        self.interface.increaseMotorAngleReferences(self.motors, [angle, -angle])

        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.1)

    def turnSonar90(self):
        radianPerDegre2 = 0.03405
        angle = 45 * radianPerDegre2

        self.interface.increaseMotorAngleReferences([self.motor3], [angle])

        while not self.interface.motorAngleReferencesReached([self.motor3]) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.1)

    def turnSonar90Neg(self):
        radianPerDegre2 = 0.03405
        angle = -45 * radianPerDegre2

        self.interface.increaseMotorAngleReferences([self.motor3], [angle])

        while not self.interface.motorAngleReferencesReached([self.motor3]) :
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
        sumThetaX = 0
        sumThetaY = 0
        weight=0
        pi = 3.14159265359
        
        print (self.particleSet)

        for particle in self.particleSet:
            sumX=sumX+particle[0]*particle[3]
            sumY=sumY+particle[1]*particle[3]
            sumThetaX += math.cos((particle[2]/360)*2*pi) * particle[3]
            sumThetaY += math.sin((particle[2]/360)*2*pi) * particle[3]
            weight = weight + particle[3]

        sumThetaX /= weight
        sumThetaY /= weight

        return (sumX/weight, sumY/weight, (math.atan2(sumThetaY , sumThetaX)/(pi))*180)


    def go(self, distCm):
        ePer10cmSqt=0.2
        fPer10cmSqt=0.2

        ePer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)
        fPer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)

        D=distCm
        newparticleSet=[]

        self.goStraight(D)

        for particle in self.particleSet:
            error = random.gauss(0,ePer10cm)
            x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
            y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
            theta = (particle[2]+random.gauss(0.0,fPer10cm))%360
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
            new_angle = (particle[2] + angle + random.gauss(0.0,gPer90))%360
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

        st_dev = 4.0
        K = 0.0005    # fat tails
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
            return usReading[index]


    def updateParticles(self):
        z = self.getSonarReading()
        t = 4.1
        z = z+t

        newParticles=[]

        for particle in self.particleSet:
            (x,y,theta,w)= particle
            likelihood = self.calculate_likelihood(x, y, theta, z)
            newParticles.append((x,y,theta,w*likelihood))


        self.particleSet=newParticles
        self.normalise()
        self.resampling()


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
        ePer10cmSqt=1.5
        fPer10cmSqt=0.2

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
        gPer90Sqt = 2.0

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

    def angle_to_target(self, destX, destY, currentX, currentY, currentTheta):
        pi = 3.14159265359

        dx = destX - currentX
        dy = destY - currentY

        alpha = math.atan2(dy,dx)
        beta = alpha - currentTheta/360*2*pi

        if beta > pi:
            beta = beta - 2*pi
        elif beta < -pi:
            beta = beta + 2*pi
        return beta

    def targetReached(self,destX, destY, currentX, currentY):
        if (destX-10<currentX<destX+10) and (destY-10<currentY<destY+10) :
            return True
        return False

    
    
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
            
            currentposition = self.position2()
            
            i=0
            while D > 0.0:
                if (D < 7.5):
                    print "Target Reached"
                    break
                self.go2(min(D, 20))
                                   

                self.particle.update(self.particleSet)
                self.particle.draw()
                
                if (destX == 138 and destY == 168):
                    self.rotate2(90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    self.rotate2(-90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                i=i+1
                
                '''
                if (destX == 114 and destY == 168):
                    self.rotate2(-90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    self.rotate2(90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    
                if (destX == 114 and destY == 84):
                    self.rotate2(90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    self.rotate2(-90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    '''

                currentposition = self.position2()

                print "Current Dest"
                print "DestX is:" + str(destX)
                print "DestY is:" + str(destY)

                print "Current Position"
                print "CurrentX is: " + str(currentposition[0])
                print "CurrentY is: " + str(currentposition[1])
                print "CurrentTheta is:" + str(currentposition[2])

                D = self.distance_to_target(destX, destY, currentposition[0], currentposition[1])
                beta = self.angle_to_target(destX, destY, currentposition[0], currentposition[1], currentposition[2])
                
                print "beta adjustment"+ str(beta/pi*180)
                
                if abs(beta)> 0.1360034842458 and D > 5:
                    print "I am adjusting to Beta rotation"
                    self.rotate2(360*beta/(2*pi))

            self.go2(D)
            currentposition = self.position2()


            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]


'''           
            
            
            
            

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
                if (D < 15):
                    print "Target Reached"
                    break
                self.go2(min(D, 15))

                print "beta" + str(beta)

                self.particle.update(self.particleSet)

                self.particle.draw()

                currentposition = self.position2()

                print "Current Dest"
                print "DestX is:" + str(destX)
                print "DEstY is:" + str(destY)

                print "Current Position"
                print currentposition[0]
                print currentposition[1]
                print currentposition[2]

                D = self.distance_to_target(destX, destY, currentposition[0], currentposition[1])
                beta = self.angle_to_target(destX, destY, currentposition[0], currentposition[1], currentposition[2])
                if abs(beta)> 0.0160034842458:
                    self.rotate2(360*beta/(2*pi))

            self.go2(D)
            currentposition = self.position2()


            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]

'''

'''
    def navigate3(self):
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

            D = self.distance_to_target(destX, destY, currentX, currentY)

            while D > 0:
                if (self.targetReached(destX, destY, currentX, currentY)):
                    break

                print "Beta" +str(beta)
                self.rotate2(360*beta/(2*pi))
                self.particle.update(self.particleSet)
                self.particle.draw()

                self.go2(min(D, 15))
                self.particle.update(self.particleSet)
                self.particle.draw()

                currentposition = self.position2()

                print "Current Position"
                print currentposition[0]
                print currentposition[1]
                print currentposition[2]

                D = self.distance_to_target(destX, destY, currentposition[0], currentposition[1])
                beta = self.angle_to_target(destX, destY, currentposition[0], currentposition[1], currentposition[2])
                print "I want to rotoate"
                print "beta" + str(beta)

            currentposition = self.position2()

            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]
'''
'''

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

        for (destX, destY) in points:

            dx = destX - currentX
            dy = destY - currentY

            alpha = math.atan2(dy,dx)
            beta = alpha - currentTheta/360*2*pi

            if beta > pi:
                beta = beta - 2*pi
            elif beta < -pi:
                beta = beta + 2*pi



            #self.particle.update(self.particleSet)
            self.particle.draw()
            print(self.particle)

            D = self.distance_to_target(destX, destY, currentX, currentY)
            while D > 0:
                if (D < 10):
                    break
                self.rotate2(360*beta/(2*pi))
                self.go2(min(D, 10))
                self.particle.draw()

                self.particle.update(self.particleSet)
                self.particle.draw()

                currentposition = self.position2()
                D = self.distance_to_target(destX, destY, currentposition[0], currentposition[1])
                beta = self.angle_to_target(destX, destY, currentposition[0], currentposition[1], currentposition[2])

            self.rotate2(360*beta/(2*pi))
            self.go2(D)

            currentposition = self.position2()

            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]


            '''
