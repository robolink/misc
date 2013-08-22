#!/usr/bin/python

# a lot of the math was pulled out of robolink//robolinkdriver and cleaned up for use with these calculations
import numpy
import math

SIN60 = math.sqrt(3)/2
ThreeLBFN = 13.3446648
TwoLBFN = 8.89644323

link1Length = 0.28575
link2Length = .45085  # 2 == 3 
def sind(theta):
    '''
    simple way of doing sine in degrees
    '''
    val = math.sin(math.radians(theta))
    
    #this ignores an error in python's approximation of sine
    if(abs(val) < 0.000001):
        val = 0;
    
    return val

def cosd(theta):
    '''
    simple way of doing cosine in degrees
    '''
    val = math.cos(math.radians(theta))
    
    #this ignores an error in python's approximation of cosine
    if(abs(val) < 0.000001):
        val = 0;
    
    return val

def calcJacobian(angles=[0,0,0,0,0],linklengths=[0,0]):
    '''
    Updates the Jacobian based on the current joint angles
    '''
    
    q0 = angles[0]
    q1 = angles[1]
    q2 = angles[2]
    q3 = angles[3]
    q4 = angles[4]

    link1Length = linklengths[0]
    link2Length = linklengths[1]
    
    a1 = ((-cosd(q0)*cosd(q1)*cosd(q2)+sind(q0)*sind(q2))*sind(q3)-cosd(q0)*sind(q1)*cosd(q3))*link2Length-cosd(q0)*sind(q1)*link1Length
    a2 = (sind(q0)*sind(q1)*cosd(q2)*sind(q3)-sind(q0)*cosd(q1)*cosd(q3))*link2Length-sind(q0)*cosd(q1)*link1Length
    a3 = (sind(q0)*cosd(q1)*sind(q2)-cosd(q0)*cosd(q2))*sind(q3)*link2Length
    a4 = ((-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*cosd(q3)+sind(q0)*sind(q1)*sind(q3))*link2Length
    a5 = 0
    
    b1 = ((-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*sind(q3)-sind(q0)*sind(q1)*cosd(q3))*link2Length-sind(q0)*sind(q1)*link1Length
    b2 = (-cosd(q0)*sind(q1)*cosd(q2)*sind(q3)+cosd(q0)*cosd(q1)*cosd(q3))*link2Length+cosd(q0)*cosd(q1)*link1Length
    b3 = (-cosd(q0)*cosd(q1)*sind(q2)-sind(q0)*cosd(q2))*sind(q3)*link2Length
    b4 = ((cosd(q0)*cosd(q1)*cosd(q2)-sind(q0)*sind(q2))*cosd(q3)-cosd(q0)*sind(q1)*sind(q3))*link2Length
    b5 = 0
    
    c1 = 0
    c2 = (-cosd(q1)*cosd(q2)*sind(q3)-sind(q1)*cosd(q3))*link2Length-sind(q1)*link1Length
    c3 = sind(q1)*sind(q2)*sind(q3)*link2Length
    c4 = (-sind(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q3))*link2Length
    c5 = 0
    
    d1 = 0
    d2 = -sind(q0)*sind(q1)
    d3 = sind(q0)*cosd(q1)*sind(q2)-cosd(q0)*cosd(q2)
    d4 = (-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*sind(q3)-sind(q0)*sind(q1)*cosd(q3)
    d5 = (-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*sind(q3)-sind(q0)*sind(q1)*cosd(q3)
    
    e1 = 0
    e2 = cosd(q0)*sind(q1)
    e3 = -cosd(q0)*cosd(q1)*sind(q2)-sind(q0)*cosd(q2)
    e4 = (cosd(q0)*cosd(q1)*cosd(q2)-sind(q0)*sind(q2))*sind(q3)+cosd(q0)*sind(q1)*cosd(q3)
    e5 = (cosd(q0)*cosd(q1)*cosd(q2)-sind(q0)*sind(q2))*sind(q3)+cosd(q0)*sind(q1)*cosd(q3)
        
    f1 = 1
    f2 = cosd(q1)
    f3 = sind(q1)*sind(q2)
    f4 = -sind(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)
    f5 = -sind(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)
    
    
    # because of the joint limitations on the arm, we're going to use position control only. this means we only use the top half of the jacobian.
    
    Jacobian = numpy.matrix([[a1, a2, a3, a4, a5],
                             [b1, b2, b3, b4, b5],
                             [c1, c2, c3, c4, c5],
                             [d1, d2, d3, d4, d5],
                             [e1, e2, e3, e4, e5],
                             [f1, f2, f3, f4, f5]])
    
    PositionJacobian = numpy.matrix([[a1, a2, a3, a4, a5],
                                     [b1, b2, b3, b4, b5],
                                     [c1, c2, c3, c4, c5]])

    return Jacobian


def tau(inval):
    return numpy.matrix([
        [inval],
        [inval],
        [inval],
        [inval],
        [inval],
        ])

def fv(vlist):
    return numpy.matrix([
        [vlist[0]],
        [vlist[1]],
        [vlist[2]],
        [vlist[3]],
        [vlist[4]],
        [vlist[5]],
        ])

def compareJacob(torquevector,forcevector,angles,index):
    jac = calcJacobian(angles,[link1Length,link2Length])
    torque2 = jac.transpose() * forcevector
    if (torquevector >= torque2).all():
        print index,(torquevector >= torque2).all()
    
def main():
    nominal = tau(.0827)
    stalling = tau(.782)
    two = fv([ThreeLBFN*SIN60, 0, ThreeLBFN/2, 0, 0, 0])
    three = fv([TwoLBFN*SIN60, 0, TwoLBFN/2, 0, 0, 0])
    
    ### Conf 1
    print "Config 1:"
    print "0,90,i,90,0"
    
    c1range = range(0,121)
    
    print "Nominal @ 3LBF"
    for i in c1range:
        compareJacob(nominal,three,[0,90,i,90,0],i)
    print "Nominal @ 2LBF"
    for i in c1range:
        compareJacob(nominal,two,[0,90,i,90,0],i)
    print "Stalling @ 3LBF"
    for i in c1range:
        compareJacob(stalling,three,[0,90,i,90,0],i)
    print "Stalling @ 2LBF"
    for i in c1range:
        compareJacob(stalling,two,[0,90,i,90,0],i)
        
        
    ### Conf 2
    print "Config 2:"
    print "45,i,0,j,0"
    
    c2rangeA = range(40,71)
    c2rangeB = range(120,151)
    
    print "Nominal @ 3LBF"
    for i in c2rangeA:
        for j in c2rangeB:
            compareJacob(nominal,three,[45,i,0,j,0],(i,j))
    print "Nominal @ 2LBF"
    for i in c2rangeA:
        for j in c2rangeB:
            compareJacob(nominal,two,[45,i,0,j,0],(i,j))
    print "Stalling @ 3LBF"
    for i in c2rangeA:
        for j in c2rangeB:
            compareJacob(stalling,three,[45,i,0,j,0],(i,j))
    print "Stalling @ 2LBF"
    for i in c2rangeA:
        for j in c2rangeB:
            compareJacob(stalling,two,[45,i,0,j,0],(i,j))
        
           

if __name__ == "__main__":
    main()