"""Does not work yet, do not use
"""

from math import *
from jointAngles import *
import matplotlib.pyplot as plt
from time import *

def closingArea(x1,y1,z1,x2,y2,z2):
    """

    """
    prevTime = int(round(time()*1000)) #ms
    print 'start time', prevTime
    workSpace = armSpace(x1,y1,z1,x2,y2,z2)
    speed = radians(90.0)/1500.0 #Rad/ms
    theta = 0
    oldx = x1
    oldy = y1

    #Find initial parameters
    l1 = lineLength([x1, y1], [0, 0])
    print 'start length', l1
    l2 = lineLength([x2, y2], [0, 0])
    l3 = lineLength([x1, y1], [x2, y2])

    psy = acos((l1**2 + l2**2 - l3**2)/(2*l1*l2)) #Angle Origin-Point1-Point2
    phi = psy + radians(jointAngles(x1,y1,z1)[0]) #Angle of line off vertical from start to end points
    print 'phi', phi

    while True:
        currentTime = int(round(time()*1000)) #ms
        print 'current time', currentTime
        deltaTime = currentTime - prevTime #ms
        print 'delta time', deltaTime

        theta += speed*deltaTime #Degrees
        print 'angle change', theta
        totalTravel = (l1*sin(theta))/sin(pi - theta - psy)
        print 'total travel', totalTravel
        newx = oldx + totalTravel*sin(phi)
        newy = oldy + totalTravel*cos(phi)
        print 'new coords', newx, newy


        workSpace = armSpace(newx,newy,z1,x2,y2,z2)
        #Publish workspace

        oldx = newx
        oldy = newy
        prevTime = currentTime #ms

def armSpace(x1,y1,z1,x2,y2,z2):
    #print x1,y1,z1,',',x2,y2,z2
    #Find base angles
    angles1 = jointAngles(x1,y1,z1)
    angles2 = jointAngles(x2,y2,z2)
    theta1 = radians(angles1[0])
    theta2 = radians(angles2[0])

    #Convert to mm and find extension distance
    x1 = ticToMm(x1)
    y1 = ticToMm(y1)
    z1 = ticToMm(z1)
    x2 = ticToMm(x2)
    y2 = ticToMm(y2)
    z2 = ticToMm(z2)
    dist1 = sqrt(x1**2 + y1**2 + z1**2)
    dist2 = sqrt(x2**2 + y2**2 + z2**2)

    length1 = dist1 + 40 #length from base to end of arm
    length2 = dist2 + 40 #length from base to end of arm

    #print "Len 1:", length1
    #print "Len 2:", length2
    pos1 = armLocation(length1,theta1,[x1,y1])
    pos2 = armLocation(length2,theta2,[x2,y2])

    if theta1 == theta2:
        return pos1
    elif theta1 < theta2:
        return buildWorkspace(pos1, pos2)
    elif theta1 > theta2:
        return buildWorkspace(pos2, pos1)
    

def armLocation(length, theta, position = [0,0]):
    """Finds location of the arm in space relative to the base.

    length : distance from base to arm end (mm)
    theta : angle of base (rad)
    position : x,y location of the end effector
    """
    #print "Angle:",theta
    width = 263.5
    dx = 125
    dy = 40
    p1 = (position[0]+dx*cos(theta)+dy*cos(pi/2 - theta),position[1]-dx*sin(theta)+dy*sin(pi/2 - theta))
    p2 = (p1[0]-length*sin(theta),p1[1]-length*cos(theta))
    p3 = (p2[0]-width*cos(theta),p2[1]+width*sin(theta))
    p4 = (p3[0]+length*sin(theta),p3[1]+length*cos(theta))
        
    plt.plot([p1[0], p2[0], p3[0], p4[0], p1[0]], [p1[1], p2[1], p3[1], p4[1], p1[1]])
    plt.axis([-700, 700, -200, 700])

    return [p1, p2, p3, p4]


def buildWorkspace(start, end):
    workArea = [end[0], end[1], start[2], start[3]]
    checkPoints = [start[0], end[0], end[3]]
    startint = 0
    endint = 3
    xpoints = [end[0][0], end[1][0], start[2][0], start[3][0]]
    ypoints = [end[0][1], end[1][1], start[2][1], start[3][1]]
    while workArea[0] != workArea[-1] and len(workArea) < 8:
        #print len(workArea)
        top = appendOuter(workArea, checkPoints)
        workArea.append(top)
        xpoints.append(top[0])
        ypoints.append(top[1])
        
        checkPoints.remove(top)
        
    plt.plot(xpoints,ypoints)
    #plt.show()
    return workArea
 

def appendOuter(workArea, points):
    """
    """
    p2 = workArea[-1]
    p1 = workArea[-2]
   
    angles = {}
    
    a = lineLength(p1,p2)#Dist from point 1 to point 2
    
    for point in points:
        b = lineLength(p2, point) #Second leg
        
        h = lineLength(p1,point) #Hypotenuse
        
        ang = acos((a**2+b**2-h**2)/(2*a*b))
        
        angles[ang] = point
    #print 'ang', angles
    #print ''
    return angles[max(angles)]


def lineLength(p1,p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def main():
    closingArea(-4000,4000,0,0,6000,0)


if __name__ == '__main__':
    main()
