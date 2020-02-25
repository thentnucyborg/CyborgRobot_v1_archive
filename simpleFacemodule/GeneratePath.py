# -*- coding: utf-8 -*-
"""
Created on Wed Nov 25 16:07:41 2015

@author: haavarws
"""

import math
import random

"""
Path simulator.

Inputs: Start and end position [x,y,z] cordinates, and number of steps

Output: A list with a number of entries equal to steps, giving a linear
path from start to end.

Could be used to create more complex paths by combining linear segments.

"""
def pathSteps(startpos, endpos, steps):
    
    path = [[]]
    path[0] = startpos.copy()
    
    
    distance = distance_calc(startpos, endpos)
    

    stepsize = stepsize_calc(distance,steps)
    
    
    position = startpos.copy()
    
    for i in range(steps):
        position[0] = position[0] + stepsize[0]
        position[1] = position[1] + stepsize[1]
        position[2] = position[2] + stepsize[2]
        
        path.append(position.copy())
        
    return path
    
    
def pathSpeed(startPos,endPos,speed):
    path = [[]]
    path[0] = startPos.copy()
    
    
    distance = distance_calc(startPos, endPos)
    absDist = math.sqrt(distance[0]**2 + distance[1]**2 + distance[2]**2)
    steps = math.floor(absDist/speed)    

    stepsize = stepsize_calc(distance,steps)
    
    
    position = startPos.copy()
    
    for i in range(steps):
        position[0] = position[0] + stepsize[0]
        position[1] = position[1] + stepsize[1]
        position[2] = position[2] + stepsize[2]
        
        path.append(position.copy())
        
    return path

def moveThrough(enviroment,speed):
    
    xMax,xMin,yMax,yMin = enviroment    
    
    choice = random.random()
    
    height = random.uniform(1.5,2)
    
    if choice < 0.25:
        xStart = xMin
        yStart = random.uniform(yMin,yMax)
        
        xEnd = xMax
        yEnd = random.uniform(yMin,yMax)
        
    elif choice < 0.5:
        xStart = random.uniform(xMin,xMax)
        yStart = yMin
        
        xEnd = random.uniform(xMin,xMax)
        yEnd = yMax
    elif choice < 0.75:
        xStart = xMax
        yStart = random.uniform(yMin,yMax)
        
        xEnd = xMin
        yEnd = random.uniform(yMin,yMax)
    else:
        xStart = random.uniform(xMin,xMax)
        yStart = yMax
        
        xEnd = random.uniform(xMin,xMax)
        yEnd = yMin
        
    path = pathSpeed([xStart,yStart,height],[xEnd,yEnd,height],speed)
    
    return path
    
def randomMovement(enviroment, moves, personHeight, speed):
    xMax,xMin,yMax,yMin = enviroment    

    path = []

    xPrev = random.uniform(xMin,xMax)
    yPrev = random.uniform(yMin, yMax)
    
    
    for i in range(moves):
        x = random.uniform(xMin,xMax)
        y = random.uniform(yMin, yMax)

        
        movement = pathSpeed([xPrev,yPrev,personHeight],[x,y,personHeight],speed)
        path.extend(movement)
        
        xPrev = x
        yPrev = y
        
    return path
        
def moveAndLinger(startPos,endPos,speed, timesLingering):
    
    path = pathSpeed(startPos,endPos,speed)
    
    for i in range(timesLingering):
        path.append(endPos.copy())
    
    return path

def approachLingerLeave(speed, startPos, endPos, exitPos, timesLingering):
    

    path = moveAndLinger(startPos,endPos,speed,timesLingering)

    path.extend(pathSpeed(endPos,exitPos,speed))    
    
    return path
    

def getRandomPosAtEdge(xMax,xMin,yMax,yMin):
    
    choice = random.random()
    
    if choice < 0.25:
        xStart = xMin
        yStart = random.uniform(yMin,yMax)
    
    elif choice < 0.5:
        xStart = random.uniform(xMin,xMax)
        yStart = yMin

    elif choice < 0.75:
        xStart = xMax
        yStart = random.uniform(yMin,yMax)

    else:
        xStart = random.uniform(xMin,xMax)
        yStart = yMax
    
    return [xStart,yStart]

    

def distance_calc(start, end):
    distance = []    
    
    distance.append( end[0] - start[0])
    distance.append( end[1] - start[1])
    distance.append( end[2] - start[2])
    
    return distance


def stepsize_calc(distance,steps):
    stepsize = []
    
    # Avoids division on 0
    if steps is 0: 
        steps = 1
    
    stepsize.append( distance[0] / steps)
    stepsize.append( distance[1] / steps)
    stepsize.append( distance[2] / steps)
    
    return stepsize
    


"""
Input transformation
Transforms the Position cordinates relative to the cyborg to distance, and angles
along the x-y and z-(x,y) planes

"""

def person_distance_calc(position):
    
    return math.sqrt(position[0]**2 + position[1]**2)

#angle from the front of robot
def theta_calc(position):
    
    if position[0] == 0:
        return 0
        
    elif position[1] == 0:
        return math.pi/2
        
    else:    
        return math.atan(position[0]/position[1])
    
#angle from robot
def phi_calc(position, robot_height):   
    distance = person_distance_calc(position)
    
    if distance == 0:
        return math.pi / 2
        
    elif (position[2] - robot_height) == 0:
        return 0
        
    else:
        return math.atan( (position[2] - robot_height)/ distance)
    
    
def position_transform(position, robot_height):
    trans = []
    
    for pos in position:
        
        dist = person_distance_calc(pos)
        theta = theta_calc(pos)
        phi = phi_calc(pos, robot_height)
        
        trans.append([dist,theta,phi])
        
    return trans
    
    
