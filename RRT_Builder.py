import numpy as np
import random
from scipy import spatial
import math
import matplotlib.pyplot as plt


def rand_samp_build():
    start = [0,0,0,0,0,0,0]
    goal = [1,1,1,1,1,1,1]
    RRT(start,goal)

def RRT(start, goal):
    #Setting up tree
    moves = 50 # Number of moves
    dist = .2    # Total euclidean radian movement of 7 joints
    small_step = [.001, .001, .001, .001, .001, .001, .001]
    dict_RRT = {1: 0}
    
    array_good_pts = build_data()
    array_good_pts = np.vstack([goal, array_good_pts])  
    tree_good_pts = spatial.KDTree(array_good_pts)

    array_RRT = np.vstack([start, small_step])      
    tree_RRT = spatial.KDTree(array_RRT)

    for i in range(0, moves):
        print "# of moves so far:", i*2
        ##############################################################################
        #q_rand <-- RANDOM_CONFIG()
        ##############################################################################
        #Choose random point on the RRT, extend it towards NN of tree_good_pts
        temp0 = random.randint(0, len(array_RRT)-1)
        q_recent = array_RRT[temp0]
        temp1 = tree_good_pts.query(q_recent, 1, 0, 2, 5)
        temp2 = temp1[1]
        q_rand = array_good_pts[temp2]

        #delete point out of tree of random points
        array_good_pts = np.delete(array_good_pts, temp2, 0)
        tree_good_pts = spatial.KDTree(array_good_pts)

        if collision_line(q_recent, q_rand)==1:
            #add this random point to the RRT, and log parent
            #print "this should happen k times"
            array_RRT = np.vstack([array_RRT, q_rand])
            tree_RRT = spatial.KDTree(array_RRT)
            dict_RRT[len(array_RRT)-1] = temp0
            reached = within_dist(q_rand, goal)

        if reached == 1:

            path = getPath(dict_RRT, array_RRT)
            plotter(array_RRT,1) #only works for 2D
            return array_RRT

        ##############################################################################
        #EXTEND (T, q_rand)
        ##############################################################################
        temp3 = extend(tree_good_pts, array_RRT, goal, tree_RRT)

        if temp3[3]:
            array_RRT = temp3[0]
            tree_RRT = spatial.KDTree(array_RRT)
            dict_RRT[len(array_RRT)-1] = temp3[4]

        if temp3[2]:

            path = getPath(dict_RRT, array_RRT)

            plotter(array_RRT,1) #only works for 2D
            return array_RRT

        array_RRT = temp3[0]
        q_recent = temp3[1]

    print array_RRT
    path = getPath(dict_RRT, array_RRT)
    print path
    plotter(array_RRT,1)

    return array_RRT

def extend(tree_good_pts, array_RRT, q_goal, tree_RRT):
    temp1 = tree_RRT.query(q_goal, 1, 0, 2, 5)
    parent_index = temp1[1]
    q_near = array_RRT[parent_index]

    disp = (q_goal-q_near)   #total displacement to goal from current position
    for i in range(0,2):
         temp = math.sqrt(math.pow(disp[i],2))
    mag_move = math.sqrt(temp) 

    if mag_move == 0:
         return array_RRT, q_near, 0, 0

    move = .1*disp/mag_move

    #Thus the closest neighbor of the goal point already in the RRT has been computed
    #We must collision check it before adding it to the RRT
    q_new = q_near + move
      
    #collision check line point here, for now we will assume true
    if collision_line(q_near, q_new):
        # T.add_vertex(q_new), T.add_edge is done here too
        array_RRT = np.vstack([array_RRT, q_new])

        #see if close enough to the goal position
        dist = within_dist(q_new, q_goal)
        if dist:
            return array_RRT, q_near, 1, 1, parent_index               #3rd position is that it got to goal.
                                                        #4th position is that it passed collision
        #if it is not, return advanced
        return array_RRT, q_near, 0, 1, parent_index

    #if collision checked line is bad
    return array_RRT, q_near, 0, 0, parent_index


def build_data():       
    k = 1000 #Number of Random Points
    A = sampler()
    
    for i in range(0, k):
        newrow = sampler()
          #Check collision here in an if statement
        A = np.vstack([A, newrow])


    return A

def getPath(dict_RRT, array_RRT):
    x = len(array_RRT)-1
    rev_indices_path = x

    while x > 0:
        x = dict_RRT[x]
        rev_indices_path = np.vstack([rev_indices_path, x])

    indices_path = rev_indices_path[::-1]

    print "indices path:", indices_path

    #######################
    y =len(indices_path)
    path = array_RRT[0]

    for i in range (1, y):
        path = np.vstack([path, array_RRT[indices_path[i]]])

    print "path:", path

    path = smoothPath(path)
    print "smoothed path:", path

    return indices_path, path

def smoothPath(path):
    j = 0
    iterations = 0

    while ((j < 2) and (len(path)>2)):          #(j < 2) and (len(path)>2)
        iterations = iterations + 1 

        x = [-1]
        y = [-1]

        for i in xrange(0,len(path)-2,2):
            check = collision_line(path[i], path[i+2])
            if check == 0:
                j = j + 1
            if check:
                x = np.vstack([x, i+1])

        for i in range(len(x)-1, 0, -1):
            path = np.delete(path, x[i], 0)

        if len(path) == 2:
            return path
   
        for i in xrange(1, len(path)-2,2):
            check = collision_line(path[i], path[i+2])
            if check == 0:
                j = j + 1
            if check:
                x = np.vstack([y, i+1])

        for i in range(len(y)-1, 0, -1):
            path = np.delete(path, y[i], 0)
   
    return path

#Checks line of point 1 and point 2 for collision in increments of .2 radians
def collision_line(pt1, pt2):
    disp = (pt2 - pt1)   #total displacement to goal from current position
    for i in range(0,7):
        temp = math.sqrt(math.pow(disp[i],2))
    mag_move = math.sqrt(temp) 

    #incremental testing move found

    if mag_move == 0:
        return 1

    increment = .2*disp/mag_move

    for i in range(1,6):
        check_pt = disp+i*increment
        if 0:               #collision check check_pt here
            return 0

    return 1

def within_dist(q_near, q_goal):
    dist_check = .1

    sum = 0
    for i in range(0,7):
        temp = q_goal[i] - q_near[i]
        temp2 = math.pow(temp,2)
        sum = sum + temp2

    dist = math.sqrt(sum)

    if (dist < dist_check) or (np.array_equal(q_near,q_goal)==1):
        return 1
    return 0

    #Returns a single non-collision-checked sample with 7 completely random joint variables
def sampler():
    '''
    The following j values represent the minimum angle in radians of the joint variable theta.
    Note: .02 rad has been added to ensure that joint limits are not hit. 
    '''
    S0 = -2.441
    S1 = -2.127
    E0 = -3.008
    E1 = -.032
    W0 = -3.039
    W1 = -1.551
    W2 = -3.039
    
    j_min =[S0, S1, E0, E1, W0, W1, W2]
    rand_array = [0,0,0,0,0,0,0]

    for i in range(0, 7):   #change to 7
        y = random.uniform(0,1)
        rand_array[i] = y
    '''
    Range of each joint variable (.04 rad off full range to ensure limits are not hit)
    '''
    S0_r = 3.311
    S1_r = 3.154
    E0_r = 6.016
    E1_r = 2.63
    W0_r = 6.077
    W1_r = 3.625
    W2_r = 6.077
        
    j_range = [S0_r,S1_r, E0_r,E1_r,W0_r,W1_r,W2_r]
    
    '''
    Now, multiply rand*range to get possible motion and add the offset j_min
    '''
    j_rand_position = [0,0,0,0,0,0,0]
    
    for i in range(0,7):
        j_rand_position[i] = rand_array[i]*j_range[i]+j_min[i]

    #Collision checker starts here
    #x = collision_server('left', j_rand_position)   #close

    return j_rand_position

def plotter(array_RRT,which_plot):
    x = len(array_RRT)

    x_data = [0]
    y_data = [0]

    for i in range(1,x-1):
        x_data = np.append(x_data, array_RRT[i][0])   
        y_data = np.append(y_data, array_RRT[i][1])   

    print "Graphing."

    plt.plot([x_data], [y_data], 'ro')
    plt.axis([-1, 2, -1, 2])
    plt.show()

    return

rand_samp_build()