# -*- coding: utf-8 -*-
"""
Created on Fri Oct 16 11:54:36 2015

@author: haavarws
"""
import Face
import GeneratePath
import graphics

import math
import time
import random
import matplotlib.pyplot as plt

"""
Path simulator.

Inputs: Start and end position [x,y,z] cordinates, and number of steps

Output: A list with a number of entries equal to steps, giving a linear
path from start to end.

Could be used to create more complex paths by combining linear segments.

"""


"""
    Neural net
"""

def printNetInOut(inp,out, position=[0,0,0], speed=0):
    #strInp = map(str,inp)
    #strOut = map(str,out)
    strInp = ["%.3f" % e for e in inp]
    strOut = ["%.3f" % e for e in out]
    print("Position: [{1:.2f},{2:.2f},{3:.2f}], Speed: {0}".format(speed, *position))
    print("Inputs: Distance: {}, Theta: {}, Phi: {}".format(*strInp))
    print("Smile Degree: {}, Pupil offset: [{},{}], Frowning {} \n".format(*strOut))



"""
simulation and visualisation

"""
def createFigure(enviroment, distancePreferences,fig,ax):
    xMax,xMin,yMax,yMin = enviroment
    careDist,goodDist = distancePreferences
    
    plt.axis([xMin ,xMax,yMin,yMax])
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    

    # Plots the circles that show the face's prefered distance

    prefDistCircle = plt.Circle((0,0),radius = goodDist, fill=False, label="Interaksjon",edgecolor="g")
    ax.add_patch(prefDistCircle)
    
    tooCloseCircle = plt.Circle((0,0),radius = careDist, fill=False, label="Oppdagelse",edgecolor="r")
    ax.add_patch(tooCloseCircle)

    plt.scatter(0,enviroment[3],color="y",label="Robot position")
    
    ax.legend()
    plt.show()


def iterateNetThroughPath(path, trans, speed, network, center, win, updateTime ):
    
    for i in range(len(trans)):
        
        
        plt.scatter(path[i][0],path[i][1],label="Posisjon")
        plt.draw()        
        
        faceParam = network.activate(trans[i])
        smileDeg, pupilx, pupily, eyebrow = faceParam
        
        Face.drawFace(center,smileDeg, pupilx, pupily, 0, eyebrow, win )
        
        win.update()
        
        printNetInOut(trans[i],faceParam, path[i],speed)
        
        time.sleep(updateTime)


def testFace(enviroment,tests,speedPreferences, distancePreferences, updateTime, robotHeight, network):
    
    xMax,xMin,yMax,yMin = enviroment
    careDist,goodDist = distancePreferences
    slowSpeed, prefSpeed, fastSpeed = speedPreferences
    
    random.seed()
    """
        Sets up the window for displaying the parametrized face
    """
    center = graphics.Point(200,200)
    win = graphics.GraphWin('Face', 400, 400,autoflush=False) # give title and dimensions
    win.setBackground('white')
    win.setCoords(0, 0, 400, 400)
    
    
    """
        Sets up the window for plotting the position of the person the face is looking at
    """
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    createFigure(enviroment, distancePreferences,fig,ax)    
    
    
    message = graphics.Text(graphics.Point(200, 380), 'Click to start simulation.')
    message.draw(win)
    win.getMouse()
    message.undraw()
    win.update()
    
    
    

    

    
    lingerTestPath = []
    # Linger at comfortable distance
    for i in range(tests.get("lingerFar")):
        z = random.uniform(1.5,2)
        
        startx = random.uniform(xMin,xMax)
        starty = yMax
        
        dist = random.uniform(goodDist,careDist)   
                
        endx = random.uniform(-dist,dist)
        endy = math.sqrt(dist**2 - endx**2)        
        
        
        timesLingering = random.randrange(6,15)
        
        lingerTestPath.extend(GeneratePath.moveAndLinger([startx,starty,z],[endx,endy,z],prefSpeed,timesLingering))
            
    #linger too close
    for i in range(tests.get("lingerClose")):
        z = random.uniform(1.5,2)
        startx = random.uniform(xMin,xMax)
        starty = yMax
        
        #dist = random.uniform(enviroment[3],tooClose)
       # endx = random.uniform(-dist,dist)
        #endy = math.sqrt(dist**2 - endx**2)  
        endx = 0
        endy = 1        
        timesLingering = random.randrange(6,15)
        
        lingerTestPath.extend(GeneratePath.moveAndLinger([startx,starty,z],[endx,endy,z],prefSpeed,timesLingering))
    
    

    
    trans = GeneratePath.position_transform(lingerTestPath,robotHeight)    
    
    
    print("\n---------------- Lingering tests ------------------ \n\n")
    iterateNetThroughPath(lingerTestPath, trans, prefSpeed, network, center, win, updateTime )
    
    
    
    # approach, linger, leave tests
    plt.close()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    
    createFigure(enviroment, distancePreferences,fig,ax)
    


    approachLingerLeavePath = []
    for i in range(tests.get("approachLingerLeaveClose")):
        z = random.uniform(1.5,2)
        startx = random.uniform(xMin,xMax)
        starty = yMax

        stoppx = 0
        stoppy = 1        
        timesLingering = random.randrange(6,15)
        

        exitx = random.uniform(xMin,xMax)
        exity = yMax

        approachLingerLeavePath.extend(GeneratePath.approachLingerLeave(prefSpeed, [startx,starty,z], [stoppx,stoppy,z], [exitx,exity,z], timesLingering))


    trans = GeneratePath.position_transform(approachLingerLeavePath,robotHeight)    

    
    print("\n---------------- Approach Linger Leave Close tests ------------------ \n\n")
    iterateNetThroughPath(approachLingerLeavePath, trans, prefSpeed, network, center, win, updateTime )
    
    message.draw(win)
    win.getMouse()
    message.undraw()
    win.update()
        
    
    plt.close()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    
    createFigure(enviroment, distancePreferences,fig,ax)




    approachLingerLeavePath = []
    
    for i in range(tests.get("approachLingerLeaveFar")):
        z = random.uniform(1.5,2)
        startx = random.uniform(xMin,xMax)
        starty = yMax
        
        dist = random.uniform(goodDist,careDist)
        stoppx = random.uniform(-dist,dist)
        stoppy = math.sqrt(dist**2 - stoppx**2)  

        timesLingering = random.randrange(6,15)
        
        exitx = random.uniform(xMin,xMax)
        exity = yMax

        approachLingerLeavePath.extend(GeneratePath.approachLingerLeave(prefSpeed,[startx,starty,z], [stoppx,stoppy,z], [exitx,exity,z], timesLingering))
    
    
    trans = GeneratePath.position_transform(approachLingerLeavePath,robotHeight)    

    
    print("\n---------------- Approach Linger Leave Far tests ------------------ \n\n")  
    iterateNetThroughPath(approachLingerLeavePath, trans, prefSpeed, network, center, win, updateTime )
    
    message.draw(win)
    win.getMouse()
    message.undraw()
    win.update()
        
    
    plt.close()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    
    createFigure(enviroment, distancePreferences,fig,ax)

    
    #random movement tests
    path = GeneratePath.randomMovement(enviroment,tests.get("randomMove"),random.uniform(1.5,2),prefSpeed)
    trans = GeneratePath.position_transform(path,robotHeight)
    
    print("\n---------------- Random movement tests ------------------ \n\n")
    iterateNetThroughPath(path, trans, prefSpeed, network, center, win, updateTime )
    

        
        
    message = graphics.Text(graphics.Point(200, 380), 'Click to close window.')
    message.draw(win)
    win.getMouse()
    
    win.close()
    plt.close()



