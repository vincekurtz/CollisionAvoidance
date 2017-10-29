#!/usr/bin/env python

##
#
# Reinforcement-Learning Based Controller
#
##

import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sklearn.neural_network import MLPRegressor
import numpy as np
import copy

# Sensor data stored in a global variable so it can be accessed asynchronously
sensor_data = LaserScan().ranges
odom_data = Odometry().twist.twist

def update_twist(twist, q_vals):
    """
    Given Q(s,a) for a certain state choose the 
    action that mazimizes Q and move accordingly.
    """
    d_lin = 0.5  # step sizes
    d_ang = 0.5

    possible_actions = [0,1,2]  # left, straight, right
    if (q_vals == None):
        # Act completely randomly
        action = random.choice(possible_actions)
    else:
        # Act greedily w.r.t Q-function
        i = np.argmax(q_vals[0])
        action = possible_actions[i]

    if (action == 2):   # right
        twist.linear.x = d_lin/2  #slow down but keep going forward
        twist.angular.z = -d_ang
    elif (action == 0):  # left
        twist.linear.x = d_lin/2  #slow down but keep going forward
        twist.angular.z = d_ang
    elif (action == 1):  # straight
        twist.linear.x = d_lin
        twist.angular.z = 0

    return action

def crash_recovery():
    """
    Iterrupt whatever's going on and get us out of a crash,
    essentially by backing up a few steps
    """
    print("Performing Crash Recovery")
    d_lin = 0.5 # step size
    cmd = Twist()
    cmd.linear.x = -d_lin  
    for i in range(15):
        controller.publish(cmd)
        rate.sleep()

def calc_reward(state, action):
    """
    Give a scalar reward based on the last action
    and the current state.
    """
    if is_crashed():
        crash_recovery()
        return -2
    elif close_to_obstacle(state):
        return -1
    elif moved_forward(action):
        return +1
    else:
        return 0

def moved_forward(action):
    """
    Indicate if we've progressed towards the goal
    """
    if (action == 1):  # moved straight ahead
        return True
    return False

def close_to_obstacle(state):
    """
    Return true or false depending if we're within
    a certain distance of an obstacle. 
    """
    cutoff_dist = 1.0
    closest_obstacle = min(state[0])
    if (closest_obstacle < cutoff_dist):
        return True
    return False

def is_crashed():
    """
    Indicate if the robot is crashed by seeing if
    the robot is actually moving
    """
    zero_movement = Twist()  # a blank twist command
        
    if (odom_data == zero_movement):
        return True
    return False

def sensor_callback(data):
    """
    Handle new sensor data by updating a global variable
    """
    global sensor_data
    sensor_data = data.ranges   # raw numbers for each angle increment

def odom_callback(data):
    global odom_data
    odom_data = data.twist.twist

def correct_Q(action, state, reward, old_Q, next_Q):
    """
    Produce a corrected Q(s,a) estimate according to:
    Q(s,a) = R + gamma*Q(s+1,a+1)
    """
    gamma = 0.5   # weights importance of future reward
    new_Q = copy.copy(old_Q)
    new_Q[action] = reward + gamma*next_Q[action]   # action indexes are 0,1,2, corresponding to position in Q-function

    return new_Q

def main():

    # set initial command velocities to 0
    cmd = Twist()

    # initialize Q-network
    q_net = MLPRegressor(
            solver='sgd',
            activation='relu',
            hidden_layer_sizes=(100,100),
            random_state=1
            )
    
    X_rand = np.array([8*random.random() for i in range(180)]).reshape(1,-1)  # random sensor input
    y_rand = np.array([1 for i in range(3)]).reshape(1,-1)      # start with equal value on all actions

    # Train on random data initially, just so we can fit to something
    q_net.partial_fit(X_rand,y_rand)

    
    update_interval = 100
    last_action = 1
    last_state = X_rand
    old_Q = q_net.predict(X_rand)

    # initialize replay buffer
    X = X_rand  # stores states
    y = y_rand  # stores corrected Q-values values

    rospy.sleep(1)  # wait a second to be sure we have good state infos

    while not rospy.is_shutdown():
        for i in range(update_interval):
            # Sensor data updated asynchronously and stored in global var sensor_data
            # Get state (sensor info)
            state = np.array(sensor_data).reshape(1,-1)

            # calculate Q(s,a) with NN
            Q_values = q_net.predict(state)

            # Control accordingly
            action = update_twist(cmd, Q_values)

            controller.publish(cmd)

            # Get reward from last action
            R = calc_reward(state, last_action)
            print(R)

            # Calculate correct Q(s,a) from last action
            # Q(s,a) = R + gamma*Q(s+1,a+1)
            corrected_Q = correct_Q(last_action, last_state, R, old_Q[0], Q_values[0])

            # Update replay buffer with correct Q(s,a)
            X = np.vstack((X, last_state))
            y = np.vstack((y, corrected_Q))

            # remember what we did this turn so we can see its result in the next step
            last_state = state
            last_action = action  
            old_Q = Q_values

            rate.sleep()


        # Update network from replay buffer
        print("Retraining!!")
        print(X.shape,y.shape)
        q_net.fit(X,y)

if __name__=='__main__':
    try:
        # Initialize ros node and publishers/subscribers
        rospy.init_node('naive_controller', anonymous=True)
        controller = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        sensor = rospy.Subscriber('/robot_0/base_scan', LaserScan, sensor_callback)
        tracker = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, odom_callback)
        rate = rospy.Rate(10) # 10 hz

        main()
    except rospy.ROSInterruptException:
        pass