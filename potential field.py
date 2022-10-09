import numpy as np
from math import *
import matplotlib.pyplot as plt
import time
from pylab import *
from matplotlib.patches import FancyArrowPatch

def dist_calc(x1, y1, x2, y2):
    global dx, dy, degree
    dx = x2-x1
    dy = y2-y1
    distance = linalg.norm([dx,dy]) # =sqrt(dx**2+dy**2)
    degree = atan2(y2-y1,x2-x1)
    return distance, degree

def attractive(d,degree,s,r,alpha): #s=attractive potential, r=radius of goal 반경
    if d < r:
        delx = 0
        dely = 0
    elif r <=d <=s+r:
        delx = alpha*(d-r)*cos(degree)
        dely = alpha*(d-r)*sin(degree)
    elif d > s + r:
        delx = alpha*s*cos(degree)
        dely = alpha*s*sin(degree)
    return delx, dely

def repulsive(d,degree,s,r,beta): #s=repulsive 반경 , r=obstacle 반경
    if d < r:
        delx = -np.sign(cos(degree)) #np.sign(cos(degree)) = 0 or 1
        dely = -np.sign(sin(degree))
    elif r <=d <=s+r:
        delx = -beta*(s+r-d)*cos(degree+pi/2)
        dely = -beta*(s+r-d)*sin(degree+pi/2)
    elif d > s + r:
        delx = 0
        dely = 0
    return delx, dely

def draw_robot(delx,dely,robot_x,robot_y):
    #Ideally you should be using OOP so self for coordinates will be useful
    robot_x = robot_x  + delx
    robot_y = robot_y +  dely
    return robot_x, robot_y

def obstacle_maker(x,y,theta,n): #n !=0 일때 장애물 이동 , n=임의의 상수
    x = x + n*0.1*cos(theta)
    y = y + n*0.1*sin(theta)
    return x, y
    

def main():
    print (__doc__)
    iter_num = 0
    MapSize = 40
    MaxSteps = 100
    xpos, ypos = 0,0
    xgoal, ygoal = 40,40
    ObstacleNumber = 20
    RadiusOfInfluence = 4
    RadiusofObstacle = 1
    alpha = 1 #attractive gain
    beta = 0.8 #repulsive gain
    AttractivePotential = 1
    RadiusOfGoal = 1
    xobs = (MapSize)*np.random.rand(ObstacleNumber)
    yobs = (MapSize)*np.random.rand(ObstacleNumber)
    # We have to print obstacle, goal and initial position together
    robot_x = xpos
    robot_y = ypos
    robot_v = 0
    robot_angle = 0
    GoalError_x = xgoal - robot_x
    GoalError_y = ygoal - robot_y
    traj = []
    fig, ax = plt.subplots()

    TotalError = np.linalg.norm([GoalError_x,GoalError_y]) 
    while(iter_num < MaxSteps) & (TotalError>1.1):
        rep_mat = []

        for i in range(ObstacleNumber):
            xobs[i],yobs[i] = obstacle_maker(xobs[i],yobs[i],(pi*i)/(ObstacleNumber),1)
            # xobs[i]=xobs[i]+1
            # yobs[i]=yobs[i]+1


        for i in range(len(xobs)):
            [distance, degree] = dist_calc(robot_x,robot_y,xobs[i],yobs[i])
            rep_mat.append([distance, degree])
        #print rep_mat

        field_rep = []
        for i in range(ObstacleNumber):
            [delx, dely] = repulsive(rep_mat[i][0], rep_mat[i][1], RadiusOfInfluence, RadiusofObstacle, beta)
            field_rep.append([delx,dely])
        # print (field_rep)

        att_mat = []

        [distance, degree] = dist_calc(robot_x,robot_y,xgoal,ygoal)
        att_mat = [distance, degree]

        [delx, dely] = attractive(att_mat[0],att_mat[1],AttractivePotential,RadiusOfGoal,alpha)
        final_field = [delx,dely]
        field_rep.append(final_field)
        final_field = field_rep

        fin = []

        for column in range(2):
            a = sum(row[column] for row in final_field)
            fin.append(a)

        robot_x, robot_y= draw_robot(fin[0],fin[1],robot_x,robot_y) #fin[0]=x좌표이동=delx , fin[1]=y좌표이동=dely

        traj.append([robot_x,robot_y])
        GoalError_x = xgoal - robot_x
        GoalError_y = ygoal - robot_y
        TotalError = np.linalg.norm([GoalError_x,GoalError_y])

        plt.plot(xpos,ypos,'rx',xgoal,ygoal,'go',robot_x,robot_y,'ro')
        plt.axis([-5,55,-5,55])
        



        plt.plot(xobs,yobs,'b*') # show obstacle 
        
        plt.draw()

        
        plt.pause(0.2)
        
       
if __name__ == '__main__':
    main()