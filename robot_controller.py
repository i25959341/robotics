import brickpi
import time
import random
import math
from copy import deepcopy
from particleDataStructures import *
from place_rec import *

PI = 3.1416
NO_BINS = 72
NO_BINS_HIST = 50

MAX_MSRT = 250 # the maximum length up to which sonar measurements are reliable


class RobotController:
    def __init__(self, _particle):
        self.motor3 = 1
        self.motors = [2,3]
        self.port = 1
        self.initInterface()
        self.initWall()
        self.pi =3.14159265359
        self.particleSet=[]
        self.NUMBER_OF_PARTICLES= 100

        self.radianPerCm = 0.2975
        self.radianPerDegre = 0.0494
        self.radianPerDegreNeg = 0.0484
        self.particle = _particle
        self.initParticles()
        self.SigContainer = SignatureContainer()
        self.locationsSig = []
        self.locationsInvSig = []

        self.initSignatures()
        self.initInvSignatures()


    def initSignatures(self):
        for i in range(5):
            ls=self.SigContainer.read(i)
            self.locationsSig.append(ls)

    def initInvSignatures(self):
        for i in range(5):
            ls = self.convert_to_invariant(self.locationsSig[i])
            self.locationsInvSig.append(ls)

    def initInterface(self):
        self.interface = brickpi.Interface()
        self.interface.initialize()

        self.interface.motorEnable(self.motors[0])
        self.interface.motorEnable(self.motors[1])

        motorParams = self.interface.MotorAngleControllerParameters()
        #motorParams.maxRotationAcceleration = 6.0
        #motorParams.maxRotationSpeed = 13.0
        motorParams.maxRotationAcceleration = 9.0
        motorParams.maxRotationSpeed = 26.0
        
        motorParams.feedForwardGain = 255/20.0
        motorParams.minPWM = 18.0
        motorParams.pidParameters.minOutput = -255.0
        motorParams.pidParameters.maxOutput = 255.0
        motorParams.pidParameters.k_p = 180.0
        motorParams.pidParameters.k_i = 457.1428571
        motorParams.pidParameters.k_d = 8.875

        motorParams2 = self.interface.MotorAngleControllerParameters()
        #motorParams2.maxRotationAcceleration = 6.0
        #motorParams2.maxRotationSpeed = 13.0
        motorParams2.maxRotationAcceleration = 9.0
        motorParams2.maxRotationSpeed = 26.0
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
        motorParams3.maxRotationAcceleration = 20.0
        motorParams3.maxRotationSpeed = 20.0
        motorParams3.feedForwardGain = 255/20.0
        motorParams3.minPWM = 18.0
        motorParams3.pidParameters.minOutput = -255.0
        motorParams3.pidParameters.maxOutput = 255.0
        motorParams3.pidParameters.k_p = 150.0#250.0
        motorParams3.pidParameters.k_i = 600#1052.1739
        motorParams3.pidParameters.k_d = 5#11.5

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


    def initParticles(self):
        self.particleSet = []
        weight = 1.0/self.NUMBER_OF_PARTICLES
        tuple =(84,30,0,weight)
        for i in range(self.NUMBER_OF_PARTICLES):
            self.particleSet.append(tuple)


    def initParticles2(self, currentX, currentY, currentTheta):
        self.particleSet = []
        weight = 1.0/self.NUMBER_OF_PARTICLES
        tuple =(currentX,currentY,currentTheta,weight)
        for i in range(self.NUMBER_OF_PARTICLES):
            self.particleSet.append(tuple)


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


    def calculate_likelihood(self,x, y, theta, z):
        #change theta
        theta = theta/360.0 * self.pi * 2.0

        st_dev = 4.0
        K = 0.001    # fat tails
        maxAngle= 0.610865# angle after which the sonar's measurements arent reliable anymore

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

        # Computing the likelihood
        p = math.exp(-((z - hit_m)**2) / (2 * (st_dev**2))) + K

        return p


    def getSonarReading(self):
        usReading=[]
        for i in range(20):
            valSonar = self.interface.getSensorValue(self.port)[0]
            if(valSonar < 255):
                usReading.append(valSonar)
            #if not usReading :
                #print "Failed US reading"
        nbMeasures = len(usReading)
        if (nbMeasures == 0):
            return 255
        else:
            index = int(nbMeasures/2)
            usReading.sort()
            return usReading[index]


    def updateParticles(self):
        z = self.getSonarReading()
        t = 1.0
        z = z+t

        newParticles=[]

        for particle in self.particleSet:
            (x,y,theta,w)= particle
            likelihood = self.calculate_likelihood(x, y, theta, z)
            newParticles.append((x,y,theta,w*likelihood))


        self.particleSet=newParticles
        self.normalise()
        self.resampling()


    def updateParticles2(self, sonarTheta):
        z = self.getSonarReading()
        t = 1.0
        z = z+t

        newParticles=[]

        for particle in self.particleSet:
            (x,y,theta,w)= particle
            likelihood = self.calculate_likelihood(x, y, theta+sonarTheta, z)
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
        ePer10cmVar=1.5
        fPer10cmVar=0.9

        ePer10cm=math.sqrt(ePer10cmVar*abs(distCm)/10)
        fPer10cm=math.sqrt(fPer10cmVar*abs(distCm)/10)

        #D=distCm/4
        newparticleSet=[]

        #for i in range(4):
        self.goStraight(distCm)
        for particle in self.particleSet:
            error = random.gauss(0,ePer10cm)
            x = particle[0]+((distCm+error)*math.cos(particle[2]/(360)*2*3.14))
            y = particle[1]+((distCm+error)*math.sin(particle[2]/(360)*2*3.14))
            theta = (particle[2]+random.gauss(0,fPer10cm))%360
            particle = (x, y, theta, particle[3])
            newparticleSet.append(particle)

        self.particleSet = (newparticleSet)

        self.updateParticles()

    def rotate2(self, angleDeg):
        gPer90Var = 2.5

        gPer90 = math.sqrt(gPer90Var*abs(angleDeg)/90)
        newparticleSet = []


        self.turn(angleDeg)
        for particle in self.particleSet:
            new_angle = (particle[2] + angleDeg + random.gauss(0,gPer90))%360
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


    def turnSonar(self, rotation):  # used to calibrate the rotation again
        radianPerDegre2 =  0.018
        angle = rotation * radianPerDegre2

        self.interface.increaseMotorAngleReferences([self.motor3], [angle])

        while not self.interface.motorAngleReferencesReached([self.motor3]) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            time.sleep(0.05)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def characterize_location(self,index):
        # print "TODO:    You should implement the function that captures a signature."
        rotation = 360.0/NO_BINS

        new_sig = []

        distances = [[] for i in range(NO_BINS)]
        for i in range (3):
        # for each of the NO_BINS bins
            for measure in range(NO_BINS):
                # make measurement
                depth = self.getSonarReading()
                 # rotate sonar
                self.turnSonar(rotation)
                 # save measurement
                distances[measure].append(depth)

            self.turnSonar(-NO_BINS*rotation)
            time.sleep(3)

        for i in range(NO_BINS):
            distances[i].sort()
            new_sig.append(distances[i][1])

        # copy this to a signature object
        ls = LocationSignature()
        ls.sig = new_sig

        #print the signature
        ls.print_signature()

        self.SigContainer.save(ls,index)

        # return the newly created signature
        return ls

    def characterize_location_real_time(self):
        # print "TODO:    You should implement the function that captures a signature."
        rotation = 360.0/NO_BINS
        
        # get the sonar back to original position
        self.turnSonar(-NO_BINS*rotation) #-rotations*NO_BINS)

        new_sig = []
        # for each of the NO_BINS bins
        for measure in range(NO_BINS):
            # make measurement
            depth = self.getSonarReading()
            #time.sleep(1)
            # rotate sonar
            self.turnSonar(rotation)
            #time.sleep(1)
            # save measurement
            new_sig.append(depth)
        # copy this to a signature object
        ls = LocationSignature()
        ls.sig = new_sig

        

        # return the newly created signature
        return ls


    def compare_signatures(self,ls1, ls2):
        dist = 0

        for i in range(len(ls1.sig)):
            dist += (ls1.sig[i]-ls2.sig[i])**2
        return dist

    def convert_to_invariant(self, ls):
        invariantSig = [0]*NO_BINS_HIST

        for i in ls.sig:
            if (i<=240):
                invariantSig[int(i/5)] = invariantSig[int(i/5)]+1

        result = LocationSignature()
        result.sig = invariantSig
        return result

    # given two signatures of a same place, get the shift]
    def getAngle(self,sig1, sig2):
        angle = -1
        score_min = 100000000
        candidate = LocationSignature()

        # incrementally rotate teh observed sig to get the candidate
        for i in range(NO_BINS):
            candidate = self.shift(sig1, i)
            # compare candidate with saved sig
            score = self.compare_signatures(sig2, candidate)
            if (score < score_min):
                score_min = score
                angle = i

        # convert angle to degrees
        angle = angle * 360/NO_BINS

        return angle

    # returns a shifted version of sig1
    def shift(self,sig1, i):
        candidate = deepcopy(sig1)
        lengthSig = len(sig1.sig)

        for j in range(lengthSig):
            candidate.sig[(j+i) % lengthSig] = sig1.sig[j]

        return candidate

    def recognize_location_invariant(self):
        sig_obs = self.characterize_location_real_time()
        hist_obs = self.convert_to_invariant(sig_obs)

        idx_most_similar = -1
        smallest_dist = 10000000

        print "hist real "
        print hist_obs.sig

        for idx in range(5):
            hist_read = self.locationsInvSig[idx]
            print "hist read " + str(idx)
            print hist_read.sig

            dist    = self.compare_signatures(hist_read, hist_obs)

            print "This is "+ str(idx) + "at dist " + str(dist)
            if (dist < smallest_dist):
                idx_most_similar = idx
                smallest_dist=dist

        print "This is where I am: I am " + str(idx_most_similar)

        ref_sig = self.locationsSig[idx_most_similar]
        angle = self.getAngle(sig_obs, ref_sig)

        print "angle: " + str(angle)

        return [idx_most_similar, angle]

    def navigate2(self, currentX, currentY, currentTheta, idx):
        points = [
            (84, 30),
            (180, 30),
            (180, 54),
            (138, 54),
            (138, 168)
        ]

        pi = 3.14159265359
        self.initParticles2(currentX, currentY, currentTheta)

        print "Ctr-C to cancel"

        for i in range(len(points)):
            point_idx = (idx + i+1) % len(points)
            destX = points[point_idx][0]
            destY = points[point_idx][1]

            dx = destX - currentX
            dy = destY - currentY
            

            alpha = math.atan2(dy,dx)

            beta = alpha - currentTheta/360.0*2.0*pi
            
            print "CurrentX is:" + str(currentX)
            print "CurrentY is:" + str(currentY)
            print "CurrentTheta is:" + str(currentTheta)
            
            print "alpha is: " + str(alpha)
            print "beta is: " + str(beta)

            if beta > pi:
                beta = beta - 2*pi
            elif beta < -pi:
                beta = beta + 2*pi

            print "I need to rotate now to " + str((destX,destY)) + " with angle" + str(beta)
            
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
                self.go2(min(D, 25.0))

                self.particle.update(self.particleSet)
                self.particle.draw()
                   
                if (destX == 180 and destY == 30 and i%2==0):
                    self.turnSonar(90)
                    #time.sleep(0.5)
                    self.updateParticles2(90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    self.turnSonar(-90)
                

                if (destX == 138 and destY == 168 and i%2==0):
                    self.turnSonar(-90)
                    #time.sleep(0.5)
                    self.updateParticles2(-90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    self.turnSonar(90)
                        

                if (destX == 84 and destY == 30 and i%2==0):
                    self.turnSonar(-90)
                    #time.sleep(0.5)
                    self.updateParticles2(-90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    self.turnSonar(90)                    

                i=(i+1)

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

                if abs(beta)> 0.1360034842:
                    print "I am adjusting to Beta rotation"
                    self.rotate2(360*beta/(2*pi))

            self.go2(D)
            currentposition = self.position2()
            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]

    def navigate_challenge(self):
        points = [
            (84, 30),
            (180, 30),
            (180, 54),
            (138, 54),
            (138, 168),
        ]
        
        [idx, theta] = self.recognize_location_invariant()
        currentX = points[idx][0]
        currentY = points[idx][1]

        self.navigate2(currentX, currentY, -theta, idx)


    '''def navigate3(self):
        points = [
            (84, 30),
            (180, 30),
            (180, 54),
            (138, 54),
            (138, 168)
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

                if (destX == 138 and destY == 168 and i%4==0):
                    self.rotate2(90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                    self.rotate2(-90)
                    self.particle.update(self.particleSet)
                    self.particle.draw()
                i=(i+1)


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

                if abs(beta)> 0.1360034842458 and D > 7.0:
                    print "I am adjusting to Beta rotation"
                    self.rotate2(360*beta/(2*pi))

            self.go2(D)
            currentposition = self.position2()


            currentX=currentposition[0]
            currentY=currentposition[1]
            currentTheta=currentposition[2]
    '''
