#!/usr/bin/env python

##
#
# Reinforcement-Learning Based Controller
#
##

import rospy
import random
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sklearn.neural_network import MLPRegressor
from sklearn.externals import joblib
import numpy as np
import copy
import matplotlib.pyplot as plt
import subprocess
import os

# Sensor data stored in a global variable so it can be accessed asynchronously
sensor_data = LaserScan().ranges
odom_data = Odometry().twist.twist

# The network will be saved regardless of completion
q_net = None

# Collision frequencies for plotting are also global variables so we can 
# access them even after the main program is shut down
iterations = []
collision_frequencies = []
cumulative_reward = []

def update_twist(twist, q_vals):
    """
    Given Q(s,a) for a certain state choose the 
    action that mazimizes Q and move accordingly.

    Use epsilon-greedy exploration too.
    """
    d_lin = 0.5  # step sizes
    d_ang = 0.5

    r = random.random()  # draw from uniform[0,1]
    epsilon = 0.2

    possible_actions = [0,1,2]  # left, straight, right
    if (r < epsilon):
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

def teleport_random():
    """
    Teleport the robot to a new random position on map
    """
    print("teleporting!")
    x_min = -8  # bounds of the map
    x_max = 8
    y_min = -8
    y_max = 8

    # Randomly generate a pose
    cmd_pose = Pose()
    cmd_pose.position.x = random.uniform(x_min, x_max)
    cmd_pose.position.y = random.uniform(y_min, y_max)

    cmd_pose.orientation.z = random.uniform(-7,7)   # janky way of getting most of the angles from a quaternarion
    cmd_pose.orientation.w = 1

    # ... and publish it as the new pose of the robot
    teleporter.publish(cmd_pose)

def calc_reward(state, action):
    """
    Give a scalar reward based on the last action
    and the current state.
    """
    if is_crashed():
        #crash_recovery()
        teleport_random()

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

def display_plot(iters, coll_freq, cu_reward):
    """
    Display a plot of collision frequencies and cumulative reward
    """
    fig, ax1 = plt.subplots()

    ax1.plot(iters, coll_freq, 'rx')
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Number of Collisions", color='r')

    ax2 = ax1.twinx()
    ax2.plot(iters, cu_reward, 'bo')
    ax2.set_ylabel("Cumulative Reward", color='b')

    fig.tight_layout()
    #plt.save("collision_frequency_plot.png")
    plt.show()

def start_simulator(gui=True):
    with open('/home/vince/.ros/log/stage_from_rl_controller.log', 'w') as fp:
        worldfile = "/home/vince/catkin_ws/src/collision_avoidance/worlds/static.world"
        if gui:
            proc = subprocess.Popen(["rosrun", "stage_ros", "stageros", worldfile], stdout=fp)
        else:
            # Adding -g argument runs the simulator without the gui
            proc = subprocess.Popen(["rosrun", "stage_ros", "stageros", "-g", worldfile], stdout=fp)
    return proc.pid

def save_model(trained_model):
    """
    Save our parameters to a pickel file so it can
    be reloaded and used again later
    """
    filename = '/home/vince/catkin_ws/src/collision_avoidance/tmp/trained_model.pkl'
    print("\n==> Saving Model to %s" % filename)
    with open(filename, 'wb') as out:
        joblib.dump(trained_model, out)

def main():

    sim_pid = start_simulator(gui=False)

    # set initial command velocities to 0
    cmd_vel = Twist()
    cmd_pose = Pose()   # also initialize a pose for teleportation purposes

    # initialize Q-network
    global q_net
    q_net = MLPRegressor(
            solver='adam',
            activation='relu',
            hidden_layer_sizes=(100,300,100),
            random_state=1,
            warm_start=True   # reuse previous call to fit as initialization
            )

    update_interval = 1000  # how many actions to take before retraining
    
    X_rand = np.vstack([np.array([8*random.random() for i in range(180)]).reshape(1,-1) for i in range(update_interval)])  # random sensor input
    y_rand = np.vstack([np.array([1 for i in range(3)]).reshape(1,-1) for i in range(update_interval)])      # start with equal value on all actions

    # Train on random data initially, just so we can fit to something
    q_net.partial_fit(X_rand,y_rand)
    
    last_action = 1
    last_state = X_rand[-1]  # take the last randomly generated entry to be the "previous state" for initialization
    old_Q = q_net.predict(X_rand)

    # initialize replay buffer
    X = X_rand  # stores states
    y = y_rand  # stores corrected Q-values values

    # variables to plot results at the end
    global iterations
    global collision_frequencies
    global cumulative_reward
    it = 1  # iteration counter

    rospy.sleep(1)  # wait a second to be sure we have good state infos

    while not rospy.is_shutdown():
        cf = 0  # reset collision frequency counter
        cr = 0 # reset cumulative reward counter
        print("==> Running")

        for i in range(update_interval):
            # Sensor data updated asynchronously and stored in global var sensor_data
            # Get state (sensor info)
            state = np.array(sensor_data).reshape(1,-1)

            # calculate Q(s,a) with NN
            Q_values = q_net.predict(state)

            # Control accordingly
            action = update_twist(cmd_vel, Q_values)
            controller.publish(cmd_vel)

            # Get reward from last action
            R = calc_reward(state, last_action)

            # update things that keep track of results
            if (R == -2):
                cf += 1  # we collided, iterate the counter
            cr += R  # add the reward to our running total

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
        print("==> Retraining")
        # Drop old data from the replay buffer
        X = X[update_interval:]    
        y = y[update_interval:]
        print(X.shape,y.shape)
        q_net.partial_fit(X,y)

        # Move everyone back to their original positions
        #reset_positions()
        teleport_random()

        # add collision frequency data
        iterations.append(it)
        collision_frequencies.append(cf)
        cumulative_reward.append(cr)

        print("")
        print("Iteration:  %s" % iterations)
        print("Coll Freq:  %s" % collision_frequencies)
        print("Reward:     %s" % cumulative_reward)
        print("")

        it +=1

if __name__=='__main__':
    try:
        # Initialize ros node and publishers/subscribers
        rospy.init_node('rl_controller', anonymous=True)
        controller = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        teleporter = rospy.Publisher('/robot_0/cmd_pose', Pose, queue_size=10)
        sensor = rospy.Subscriber('/robot_0/base_scan', LaserScan, sensor_callback)
        tracker = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, odom_callback)
        #reset_positions = rospy.ServiceProxy('reset_positions', Empty)
        rate = rospy.Rate(100) # in hz

        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        # always display plots before quitting, even when something gets messed up
        # along the way
        display_plot(iterations, collision_frequencies, cumulative_reward) 

        # also be sure to save the model parameters
        save_model(q_net)
