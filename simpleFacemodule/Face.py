# -*- coding: utf-8 -*-
"""
Created on Sun Oct 18 16:05:05 2015

@author: haavarws
"""

#2d face

from graphics import *



    
def drawFace(center, smilingGrade, pupilx, pupily, eyeWide, frownGrade, window):
    """ 
    Draw a face in the given window. To change face just call the function again
    and draw over the old drawing. 
    
    eyeWide is how much the eyes widen. Works best with values from 1 to -0.5
    
    """
    
    head = Circle(center,60)
    head.setFill("white")
    head.draw(window)
 
    drawMouth(center,smilingGrade, window)
     
    eye = Circle(center, 9 + 3 * eyeWide) 
    eye.move(-24,18)
    eye.setFill('white')
    eye.draw(window)
    eye2 = eye.clone()
    eye2.draw(window)
    eye2.move(48, 0)

    drawPupils(center,pupilx, pupily, window)
    
    drawEyebrows(center,frownGrade,window)
 


def drawEyebrows(center, frownGrade, window):
    """ Draws a line above both eyes in the face.
    
    Frowngrade controls which direction the line is angled. Positive values 
    angle it towards the centre of the face (making it look angry), while
    negative values angle them away from the centre. 
    
    Works best with frownGrade between +-1.
    
    """
    left1 = center.clone()
    left1.move(-33,32 + 4*frownGrade)
    
    left2 = left1.clone()
    left2.move(20 + 2* frownGrade, - 8*frownGrade)    
    
    leftEyebrow = Line(left1,left2)
    leftEyebrow.draw(window)
    
    right1 = center.clone()
    right1.move(33,32 + 4*frownGrade)
    
    right2 = right1.clone()
    right2.move(-20 - 2* frownGrade, - 8*frownGrade)    
    
    rightEyebrow = Line(right1,right2) 
    rightEyebrow.draw(window)




def drawMouth(center,smilingGrade,window):
    """ Draws a smile as two connected lines.
    
    The distance in the y direction is a function of "smilingGrade". 
    Positive values give a smile, negative values give a reverse smile. 
    Best results is with smilingGrade +-1.
    
    """
    mouth_left = center.clone() # Creates the leftmost point
    mouth_left.move(-25,-20 + 10 * smilingGrade)
    
    
    mouth_centre = mouth_left.clone()
    mouth_centre.move(25, - 15 * smilingGrade)
    
    
    mouth_right = mouth_left.clone()
    mouth_right.move(50,0)
    
    smile_left = Line(mouth_left,mouth_centre)
    smile_left.draw(window)
    
    smile_right = Line(mouth_centre,mouth_right)
    smile_right.draw(window)



def drawPupils(center,pupilx, pupily, window):
    """
    Draws the pupils in the center of the eyes. pupilx and pupily gives the 
    offset from the center position, and should not be more than +-1.
    
    positive x axis goes to the right, positive y axis goes up.
    """
    pupil = Circle(center,3)
    pupil.move(-24 + pupilx*5 ,18 + pupily*5)
    pupil.setFill('black')
    pupil.draw(window)
    
    pupil2 = pupil.clone()
    pupil2.draw(window)
    pupil2.move(48,0)


def test():
        
    win = GraphWin('Face', 400, 400) # give title and dimensions
    win.setBackground('white')
    win.setCoords(0, 0, 400, 400)    
    

    center = Point(200,200)#top to bottom, increasing radius
    drawFace(center,-1,-1,1, 0 ,-0.25,win)

    message = Text(Point(200, 380), 'Click anywhere to quit.')
    message.setFill('blue')
    message.draw(win)
    
    win.getMouse()
    
    win.close()
 
