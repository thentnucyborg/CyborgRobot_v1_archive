# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 14:56:08 2015

@author: haavarws
"""

import GeneratePath

import math

from random import uniform




def pupilTracking(theta,phi):
    targetTheta = theta/(3.14/2)
    targetPhi = phi/(3.14/2)
    
    return [targetTheta, targetPhi]




def approachDataSet(samples, enviroment,robotHeight, prefSpeed, distPreferences, dataset ):
    
    xMax,xMin,yMax,yMin = enviroment
    careDist,goodDist = distPreferences
    
    for i in range(samples):
        
        z = uniform(1.5,2) #height
        
        startx = uniform(xMin,xMax)
        starty = yMax
        
        
        endx = 0
        endy = 1       
        
        path = GeneratePath.pathSpeed([startx,starty,z],[endx,endy,z],prefSpeed)

        trans = GeneratePath.position_transform(path,robotHeight)
        
        dataset.newSequence()
        
        for param in trans:
            target = []
            
            if param[0] < careDist:
                target.append(0.25) #smile
            else:
                target.append(0)
            
            target.extend(pupilTracking(param[1], param[2]))
            target.append(0) #eyebrows

            dataset.appendLinked(param,target)
        
    return dataset

def leaveDataSet(samples, enviroment,robotHeight, prefSpeed, distPreferences, dataset ):
    
    xMax,xMin,yMax,yMin = enviroment
    careDist,goodDist = distPreferences
    
    for i in range(samples):
        
        z = uniform(1.5,2) #height
        
        endx = uniform(xMin,xMax)
        endy = yMax
        
        dist = uniform(goodDist,careDist)   
                
        startx = uniform(-dist,dist)
        starty = math.sqrt(dist**2 - startx**2)        
        
        path = GeneratePath.pathSpeed([startx,starty,z],[endx,endy,z],prefSpeed)

        trans = GeneratePath.position_transform(path,robotHeight)
        
        dataset.newSequence()
        
        for k in range(len(trans)):
            target = []
            
            if k  < 2:
                target.append(0.25) #smile
            else:
                target.append(-0.5)
            
            target.extend(pupilTracking(trans[k][1], trans[k][2]))
            target.append(0) #eyebrows

            dataset.appendLinked(trans[k],target)
        
    return dataset


def waitCloseDataSet(samples, robotHeight, dataset):
    for i in range(samples):
        x = 0
        y = 1
        z = uniform(1.5,2) #height
        
        path = []
            
        
        for k in range(10):
            path.append([x,y,z])
        
        trans = GeneratePath.position_transform(path,robotHeight)
        
        dataset.newSequence()
        
        for k in range(len(trans)):

            if k < 2:
                smile = 0.25
                eyebrow = 0
            elif k < 6:
                smile = 0.25 + k/10
                eyebrow = -k/8
            else:
                smile = 0.75
                eyebrow = -6/8
            
            pupils = pupilTracking(trans[k][1], trans[k][2])
            
            dataset.appendLinked(trans[k],[smile,pupils[0], pupils[1],eyebrow])
    
    return dataset
    
def waitFarDataSet(samples,distPref,robotHeight,dataset):
    """
        Generates a dataset with samples number of samples, where each sample is a
        series of identical [x,y,z] cordinates chosen randomly between the "care" and
        "good" distance from the robot.
    """
    careDist,goodDist = distPref
    
    for i in range(samples):

        dist = uniform(goodDist,careDist)   
                
        x = uniform(-dist,dist)
        y = math.sqrt(dist**2 - x**2)        
        z = uniform(1.5,2) #height

        
        path = []
            
        
        for k in range(10):
            path.append([x,y,z])
        
        trans = GeneratePath.position_transform(path,robotHeight)
        
        dataset.newSequence()
        
        for k in range(len(trans)):

            if k < 2:
                smile = 0.25
                eyebrow = 0
            elif k < 6:
                smile = 0.25 - k/5
                eyebrow = k/8
            else:
                smile = -0.75
                eyebrow = 6/8
            
            pupils = pupilTracking(trans[k][1], trans[k][2])
            
            dataset.appendLinked(trans[k],[smile,pupils[0], pupils[1],eyebrow])
    
    return dataset    
        
