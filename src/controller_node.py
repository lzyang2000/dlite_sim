#!/usr/bin/env python
import numpy as np 
import sys
import rospy
import tf
import math
import matplotlib.pyplot as plt
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
curr_state=np.zeros(2)
desired_path=[ np.zeros([300,2]) , 0]
desired_global_path=[ np.zeros([200,5]) , 0]


tune_set=[[0.99,0],[-0.99,0],[0,0.99],[0,-0.99],[0.99,0.99],[0.99,-0.99],[-0.99,0.99],[-0.99,-0.99]]

control_cmd = Twist()

control_mode = 1 # 0 for manual 1 for auto 

def distance(c1,c2):
    return np.sqrt((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1]))

def cmd(data):
    control_cmd.linear.x = data[0]
    control_cmd.linear.y = data[1]
    pub.publish(control_cmd)
    
def dynamic_model(p,action):
    return np.array([p[0]+action[0],p[1]+action[1]] ) ,action

def cost( possible_tune, goal, times):
    return np.array([distance(goal,p[0]) for p in possible_tune])    

def step_control(target_position, next_target_position,times):
    pre_close_loop_input=np.zeros(2)
    pre_close_loop_input[0] = 0  
    pre_close_loop_input[1] =  0
    possible_tune = [dynamic_model(curr_state,np.array([pre_close_loop_input[0]+tune[0],pre_close_loop_input[1]+tune[1]])) for tune in tune_set]
    next_p,close_loop_input= possible_tune[np.argmin(cost(possible_tune,next_target_position,times))]
    cmd(close_loop_input)

def find_min_distance(c1):
    return np.argmin( np.array([distance(c1,desired_path[0][i]) for i in range(desired_path[1])]) )

def path_callback(data):
    size = len(data.poses)
    desired_path[1]=size
    for i in range(size):
        desired_path[0][i,0]=data.poses[i].pose.position.x
        desired_path[0][i,1]=data.poses[i].pose.position.y

def auto():
    times =0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if desired_path[1] != 0:
            try:
                (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
                curr_state[0]=trans[0]
                curr_state[1]=trans[1]
                print(curr_state[0],curr_state[1])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
                
            times=find_min_distance(curr_state)

            if times > desired_path[1]-1:
                break
            desired_state = desired_path[0][times]
            next_desired_state = desired_path[0][times+1]
            step_control(desired_state, next_desired_state ,times)
            rate.sleep()


def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def manual():
    rate = rospy.Rate(100)
    data = np.array([ 0.0, 0.0, 0.0])
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            if(data[0]< 0.35):
                data[0] = data[0] + 0.05
            else:
                data = data
        elif key == 'x':
            if(data[0]> -0.35):
                data[0] = data[0] - 0.05
            else: 
                data = data
        elif key == 'a':
            if(data[2]< 0.6):
                data[2] += 0.99
            else:
                data = data
        elif key == 'd':
            if(data[2]> -0.6):
                data[2] -= 0.99
            else:
                data = data
        elif key == 's':
            data = np.array([ 0.0,   0.0, 0.0])
        elif key == 'j':
            if(data[1]< 0.99):
                data[1] += 0.05
            else:
                data = data
        elif key == 'k':
            if(data[1]> -0.99):
                data[1] -= 0.05
            else:
                data = data
        
        elif (key == '\x03'):
            break
        else:
            data = data
        cmd(data)
        rate.sleep()

if __name__=='__main__':
    
    rospy.init_node('control')
    rospy.Subscriber('/path',Path,path_callback)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    size = 0
    listener = tf.TransformListener()


    if( control_mode == 1):
        auto()
    else:
        manual()