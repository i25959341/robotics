# -*- coding: utf-8 -*-
"""
Created on Thu Jan 28 16:34:01 2016

@author:
"""

import brickpi
import time
import random
import math


def initInterface(interface, motors):
    interface.initialize()

    interface.motorEnable(motors[0])
    interface.motorEnable(motors[1])

    motorParams = interface.MotorAngleControllerParameters()
    motorParams.maxRotationAcceleration = 3.0
    motorParams.maxRotationSpeed = 8.0
    motorParams.feedForwardGain = 255/20.0
    motorParams.minPWM = 18.0
    motorParams.pidParameters.minOutput = -255.0
    motorParams.pidParameters.maxOutput = 255.0
    motorParams.pidParameters.k_p = 250.0
    motorParams.pidParameters.k_i = 1009.0909
    motorParams.pidParameters.k_d = 11.0

    motorParams2 = interface.MotorAngleControllerParameters()
    motorParams2.maxRotationAcceleration = 3.0
    motorParams2.maxRotationSpeed = 8.0
    motorParams2.feedForwardGain = 255/20.0
    motorParams2.minPWM = 18.0
    motorParams2.pidParameters.minOutput = -255.0
    motorParams2.pidParameters.maxOutput = 255.0
    motorParams2.pidParameters.k_p = 250.0
    motorParams2.pidParameters.k_i = 1052.1739
    motorParams2.pidParameters.k_d = 11.5

    interface.setMotorAngleControllerParameters(motors[0],motorParams)
    interface.setMotorAngleControllerParameters(motors[1],motorParams2)


def goStraight(distCm, interface, motors):
    radianPerCm = 0.3725
    angle = distCm * radianPerCm 
    interface.increaseMotorAngleReferences(motors, [angle, angle])

    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        time.sleep(0.1)
        ## Maybe add a break after x seconds
    

def turn(angleDeg, interface, motors):
    #radianPerDegre = 0.0697
    radianPerDegre = 0.8
    
    angle = angleDeg * radianPerDegre 
    interface.increaseMotorAngleReferences(motors, [angle, -angle])

    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        time.sleep(0.1)

def initParticles(NUMBER_OF_PARTICLES):
    particleSet=[]
    weight = 1/NUMBER_OF_PARTICLES
    tuple =(0,0,0)
    for i in range(NUMBER_OF_PARTICLES):
        particleSet.append(tuple)
        
    return particleSet

def go10Cm(interface, motors, particleSet):
    ePer10cm=0.2
    fPer10cm=0.2
    D=10
    newparticleSet=[]
    
    goStraight(D, interface, motors)
    for particle in particleSet:
        error = random.gauss(0,ePer10cm)        
        x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
        y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
        theta = (particle[2]+random.gauss(0,fPer10cm))%360
        particle = (x, y, theta)
        newparticleSet.append(particle)
        
    return (newparticleSet)

            
def turn90Deg(interface, motors, particleSet):
    gPer90=1
    newparticleSet=[]
    angle = 90

    turn(angle, interface, motors)
    for particle in particleSet:
        new_angle = (particle[2] + angle + random.gauss(0,gPer90))%360
        new_particle = (particle[0], particle[1], new_angle)
        newparticleSet.append(new_particle)

    return (newparticleSet)

def scale(particleSet):
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

def position(particleSet, NUMBER_OF_PARTICLES):
    sumX=0
    sumY=0
    sumTheta=0
    
    for particle in particleSet:
        sumX=sumX+particle[0]
        sumY=sumY+particle[1]
        sumTheta=sumTheta+particle[2]
        
    sumX=sumX/NUMBER_OF_PARTICLES
    sumY=sumY/NUMBER_OF_PARTICLES
    sumTheta=(sumTheta/NUMBER_OF_PARTICLES)%360
    
    return (sumX,sumY,sumTheta)
    
    

def go(distCm,interface, motors, particleSet):
    ePer10cmSqt=0.01
    fPer10cmSqt=0.01
    
    ePer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)
    fPer10cm=math.sqrt(ePer10cmSqt*abs(distCm)/10)
    
    D=distCm
    newparticleSet=[]
    
    goStraight(D, interface, motors)
    
    for particle in particleSet:
        error = random.gauss(0,ePer10cm)        
        x = particle[0]+((D+error)*math.cos(particle[2]/(360)*2*3.14))
        y = particle[1]+((D+error)*math.sin(particle[2]/(360)*2*3.14))
        theta = (particle[2]+random.gauss(0,fPer10cm))%360
        particle = (x, y, theta)
        newparticleSet.append(particle)
        
    return (newparticleSet)
    
def rotate(angleDeg,interface, motors, particleSet):
    gPer90Sqt=0.1
    
    gPer90=math.sqrt(gPer90Sqt*abs(angleDeg)/90)
    newparticleSet=[]
    angle = angleDeg

    turn(angle, interface, motors)
    for particle in particleSet:
        new_angle = (particle[2] + angle + random.gauss(0,gPer90))%360
        new_particle = (particle[0], particle[1], new_angle)
        newparticleSet.append(new_particle)

    return (newparticleSet)
    
    
    
def navigate(interface, motors,particleSet):
    currentX=0
    currentY=0
    currentTheta=0
    
    pi=3.14159265359
    
    print "Ctr-C to cancel"
    
    while True:
        destX=input('Please Enter the X co-ordinate  ')
        destY=input('Please Enter the Y co-ordinate  ')
        
        dx = destX - currentX
        dy = destY - currentY
        
        
        D = math.sqrt(dx*dx+dy*dy)
        alpha = math.atan2(dy,dx)
        
        beta = alpha - currentTheta/360*2*pi
    
        if beta > pi:
            beta = beta - 2*pi
        elif beta<=-pi:
            beta = beta + 2*pi

        newparticleSet = rotate(360*beta/(2*pi),interface, motors, particleSet)
        newparticleSet2 = go(D,interface, motors, newparticleSet)
    
        currentposition = position(newparticleSet2,100)
        
        currentX=currentposition[0]
        currentY=currentposition[1]
        currentTheta=currentposition[2]


        