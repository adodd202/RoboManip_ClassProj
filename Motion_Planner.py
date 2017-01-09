#!/usr/bin/env python

import sys
import numpy as np
import random
import math
import pylab as pl
from matplotlib import collections  as mc
import matplotlib.pyplot as plt

blocks_pos = np.array([[.85,.15], [.9,0], [.8,.26], [.8,-.14], [.87, -.18]])
r_ob = .0315
num_blocks = blocks_pos.size/2
x_ob = -.35
y_ob = -.03

def motion_solver():
    convert_to_andrew_coor()
    '''
    All distances in meters
    Pt 1 is the goal side of line segment
    Launch_angle corresponds to angle that the ball comes into goal. 
        45 degrees max
       -45 degrees min
    x_dist_goal: X distance from throwing position to goal
    '''
    #Point in goal
    x1 = 0
    y1 = np.arange(.1, .2, .01)

    entrance_angle = np.arange(-.8, .8, 0.1)

    Best_motion_works = 0
    Best_num_bounce =   0
    Best_x2 =           0
    Best_y2 =           0
    Best_x_bounce =     0
    Best_y_bounce =     0 
    Best_D =            0
    Best_y1 =           0
    Best_entrance_angle = 0

    for y1_temp in np.nditer(y1):
        for entrance_angle_temp in np.nditer(entrance_angle):
            print "BEGINNING TO CHECK NEXT RAY"
            potential_motion = motion_checker(x1,y1_temp,entrance_angle_temp)

            motion_works = potential_motion[0]
            num_bounce =   potential_motion[1]
            x2 =           potential_motion[2]
            y2 =           potential_motion[3]
            x_bounce =     potential_motion[4]
            y_bounce =     potential_motion[5] 
            D =            potential_motion[6]

            if motion_works and D > Best_D:
                Best_motion_works = motion_works
                Best_num_bounce =   num_bounce
                Best_x2 =           x2
                Best_y2 =           y2
                Best_x_bounce =     x_bounce
                Best_y_bounce =     y_bounce
                Best_D =            D
                Best_y1 =           y1_temp
                Best_entrance_angle = entrance_angle_temp

    print "*****************FINAL RESULTS FOR MOVEMENT*****************"
    print "motion_works",Best_motion_works
    print "num_bounce", Best_num_bounce
    print "x1", x1
    print "y1", Best_y1
    print "x2", Best_x2
    print "y2", Best_y2
    print "x_bounce", Best_x_bounce
    print "y_bounce", Best_y_bounce
    print "D", Best_D
    print "Best_entrance_angle", Best_entrance_angle


    '''This last bit will convert from Andrew's to Sam's coordinates
    and return the results to Ashish.'''
    if Best_num_bounce == 1:
        final_slope = -math.tan(Best_entrance_angle)
    else:
        final_slope = math.tan(Best_entrance_angle)

    final_x2 =  Best_x2 + 1.2192
    final_y2 = -Best_y2 + .1475

    print "FOR ASHISH (in Sam's coor frame):"
    print "final_x2", final_x2
    print "final_y2", final_y2
    print "final_slope", final_slope

    graph(x1,Best_y1, Best_x2,Best_y2, Best_x_bounce, Best_y_bounce, Best_num_bounce)

    return final_x2,final_y2,final_slope

def motion_checker(x1, y1, entrance_angle):
    '''
    Pt2: the throw side of line segment
    x_ob, y_ob, r_ob correspond to object position/radius
    '''
    x_dist_goal = .81 # x_Dist to goal from launch line

    # This area defines the point p2. p2 is calculated from entrance angle.
    x2 = -x_dist_goal
    y2 = x_dist_goal*math.tan(entrance_angle)+y1

    print "y2 chec", y1, y2

    '''Next check if the ball can be shot with no bounces from the launch line to the goal point'''
    no_bounce1 = no_bounce(x1, y1, x2, y2)
    x2 = no_bounce1[1]
    y2 = no_bounce1[2]
    D = no_bounce1[3]
    if no_bounce1[0]:
        return True, 0, x2, y2,   0,      0,        D

    '''Next check if the ball can be shot with one bounce from the launch line to the goal point'''
    one_bounce1 = one_bounce(x_dist_goal, x1, y1, entrance_angle)
    x2 =       one_bounce1[3]
    y2 =       one_bounce1[4]
    x_bounce = one_bounce1[5]
    y_bounce = one_bounce1[6]
    D =        one_bounce1[7]

    if one_bounce1[0]:
        return True, 1, x2, y2, x_bounce, y_bounce, D

    return False,0,x2, y2, x_bounce, y_bounce, D #fix later

def no_bounce(x1, y1, x2, y2):
    '''Checks if there is a no wall bounce solution to the given line'''

    delta = [(.02135), (-.02135)]
    D = 10

    for i in range(0,num_blocks):
        for j in range(0,2):
            path_clear = line_circle_coll(x1, y1+delta[j], x2, y2+delta[j], blocks_pos[i][0], blocks_pos[i][1], r_ob)
            if path_clear[0] == False:
                return False,0,0,0
            if path_clear[1]<D:
                D = path_clear[1]

    R = intersection(line([x1,y1], [x2,y2]), line([x2,-.1], [x2,.39]))

    if R[0] == False:
        return False,0,0,0

    x = R[1]
    y = R[2]

    #If the intersection point lies along the launch line, it works.
    if y<.39 and y>-.09:
        return True,x,y,D

    return False,0,0,0

def one_bounce(x_dist_goal, x1, y1, entrance_angle):
    '''Checks if there is a one wall bounce solution to the given line'''

    '''Find x,y-coordinate of wall bounce'''
    if entrance_angle==0:
        return False,0,0,0,0,0,0,0
    if entrance_angle>0:
        y_bounce = .4855-.02135
        x_bounce = -(y_bounce-y1)/math.tan(entrance_angle)
    if entrance_angle<0:
        y_bounce = -.1905+.02135
        x_bounce = -(y_bounce-y1)/math.tan(entrance_angle)
    if (x_bounce<0 and x_bounce>-x_dist_goal) == False:
        return False,x1, y1,0,0, x_bounce, y_bounce, 0

    '''Local Variables setting up for forloop'''
    delta = [(.02135), (-.02135)]   #For top/bottom line of ball
    D = 10

    '''Collision check first ray'''
    for i in range(0,num_blocks):
        for j in range(0,2):
            path_clear = line_circle_coll(x1, y1+delta[j], x_bounce, y_bounce+delta[j], blocks_pos[i][0], blocks_pos[i][1], r_ob)
            if path_clear[0] == False:
                return False,x1, y1, 0,0, x_bounce, y_bounce, 0
            if path_clear[1]<D:
                D = path_clear[1]

    '''Solve for the second ray coordinates'''
    x2 = -x_dist_goal
    x_displacement = x_dist_goal+x_bounce #Distance from bounce to launch line
    if entrance_angle>0:
        y2 = -(math.tan(entrance_angle)*(x_bounce-x2)-y_bounce)
    if entrance_angle<0:
        y2 = math.tan(-entrance_angle)*(x_bounce-x2)+y_bounce

    R = intersection(line([x_bounce,y_bounce], [x2,y2]),  line([x2,-.1], [x2,.39]))
    intersection_exists = R[0]
    if intersection_exists == False:
        return False,x1, y1, x2, y2, x_bounce, y_bounce, 0

    '''Collision check second ray'''
    for i in range(0,num_blocks):
        for i in range(0,2):
            path_clear = line_circle_coll(x_bounce, y_bounce+delta[j], x2, y2+delta[j], blocks_pos[i][0], blocks_pos[i][1], r_ob)
            if path_clear[0] == False:
                return False,x1, y1, x2,y2, x_bounce, y_bounce, 0
            if path_clear[1]<D:
                D = path_clear[1]

    '''Return all coordinates if the end point lies in the launch line'''
    if y2<.4 and y2>-.1:
        return True, x1, y1, x2, y2, x_bounce, y_bounce, D

    return False,x1, y1, x2, y2, x_bounce, y_bounce, D

def line(p1, p2):
    '''
    convert to standard line format:
    Ax + By + C = 0
    '''
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = -(p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, C

def intersection(L1, L2):
    '''Finds the intersection of 2 lines'''
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return True,x,y
    else:
        return False,0,0

def line_circle_coll(x1, y1, x2, y2, x_ob, y_ob, r_ob):
    '''Function returns whether a collision occurs and the distance from collision'''

    '''Find distance to center of obstacle, see if within radius of obstacle'''
    arg1 = abs((y2-y1)*x_ob-(x2-x1)*y_ob+x2*y1-y2*x1)
    arg2 = math.sqrt(math.pow((y2-y1),2)+math.pow((x2-x1),2))
    D = arg1/arg2

    if D > r_ob:
        return True, D #No collision
    return False, 0 #Collision occurs

def convert_to_andrew_coor():
    for i in range(0,num_blocks):
        for j in range(0,2):
            if j == 0:
                blocks_pos[i][j] = blocks_pos[i][j]-1.2192
            else:
                blocks_pos[i][j] = -(blocks_pos[i][j]-.1475)

def graph(x1,y1, x2,y2, x_bounce, y_bounce, num_bounce):
    '''This graphs the entire playing board side and the shots being made'''
    A1x = -.81
    A1y = .4855
    A2x = 0
    A2y = .4855

    B1x = 0
    B1y = .4855
    B2x = 0
    B2y = .295

    C1x = 0
    C1y = .295
    C2x = .038
    C2y = .295

    D1x = .038
    D1y = .295
    D2x = .038
    D2y = 0

    E1x = 0
    E1y = 0
    E2x = .038
    E2y = 0

    F1x = 0
    F1y = 0
    F2x = 0
    F2y = -.1905

    G1x = 0
    G1y = -.1905
    G2x = -.81
    G2y = -.1905

    x_data = [A1x,A2x,B1x,B2x,C1x,C2x,D1x,D2x,E1x,E2x,F1x,F2x,G1x,G2x]
    y_data = [A1y,A2y,B1y,B2y,C1y,C2y,D1y,D2y,E1y,E2y,F1y,F2y,G1y,G2y]

    if num_bounce == 1:
        x_pts  = [x1, x2, x_bounce]
        y_pts  = [y1, y2, y_bounce]
    else:
        x_pts  = [x1, x2]
        y_pts  = [y1, y2]

    x_obst = []
    y_obst = []

    for i in range(0,num_blocks):
        x_obst = np.append(x_obst, blocks_pos[i][0])
        y_obst = np.append(y_obst, blocks_pos[i][1])

    plt.plot([x_data], [y_data], 'ro')
    plt.plot([x_pts], [y_pts], 'g^')
    plt.plot([x_obst], [y_obst], 'bs')

    plt.plot([.1,.1], [.6,.6], 'k:', lw=2)

    plt.axis([-1, 1, -1, 1])
    plt.show()


if __name__ == "__main__":
    motion_solver()