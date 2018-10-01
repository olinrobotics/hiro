from math import *

def jointAngles(x,y,z):
    """Returns joint angles of a STR 17 ARM

    x: int, arm units
    y: int, arm units
    z: int, arm units

    returns: list of angles (degrees)
    """
    #Converts from arm tics to mm
    x = ticToMm(x)
    y = ticToMm(y)
    z = ticToMm(z)
    

    a = sqrt(x**2 + y**2)
    b = sqrt(a**2 + z**2)
#    print b
    phi = atan2(z,a)
    psy = atan2(a,z)
    theta3 = acos(2 - b**2/375**2)
    chi = (pi - theta3)/2
#    print phi*(180/pi)
 #   print psy*(180/pi)
  #  print chi*(180/pi)

    theta3 = theta3*(180/pi) #Elbow
    theta1 = atan(x/y)*(180/pi) #Base
    theta2 = (chi + phi)*(180/pi)+90 #Shoulder
    theta4 = (chi + psy)*(180/pi) #Wrist

    return [theta1,theta2,theta3,theta4]

def ticToMm(coord):
    return coord*(355.6/3630)

def mmToTic(coord):
    return coord*(3630/355.6)

def main():
    print jointAngles(0,6000,100)

if __name__ == '__main__':
    main()