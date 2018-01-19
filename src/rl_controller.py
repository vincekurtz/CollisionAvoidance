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
from std_msgs.msg import Int8
from std_srvs.srv import Empty

import tensorflow as tf

import numpy as np
import copy
import matplotlib.pyplot as plt
import subprocess
import time

home_dir = "/home/vjkurtz/"
base_dir = "/home/vjkurtz/catkin_ws/src/collision_avoidance"

# Sensor data stored in a global variable so it can be accessed asynchronously
sensor_data = LaserScan().ranges
odom_data = Odometry().twist.twist
is_crashed = False

# Collision frequencies for plotting are also global variables so we can 
# access them even after the main program is shut down
iterations = []
collision_frequencies = []
cumulative_reward = []

######### Initialize Q-Network ################
# parameters
learning_rate = 0.001

n_hidden_1 = 100
n_hidden_2 = 300
n_hidden_3 = 100
n_input = 180   # lidar data: one distance per degree
n_classes = 3   # commands: left, right, straight

# tf graph input
X = tf.placeholder("float", [None, n_input])
Y = tf.placeholder("float", [None, n_classes])

# Layer weights and biases
weights = {
        'h1': tf.Variable(tf.random_normal([n_input, n_hidden_1])),
        'h2': tf.Variable(tf.random_normal([n_hidden_1, n_hidden_2])),
        'h3': tf.Variable(tf.random_normal([n_hidden_2, n_hidden_3])),
        'out': tf.Variable(tf.random_normal([n_hidden_3, n_classes]))
}
biases = {
        'b1': tf.Variable(tf.random_normal([n_hidden_1])),
        'b2': tf.Variable(tf.random_normal([n_hidden_2])),
        'b3': tf.Variable(tf.random_normal([n_hidden_3])),
        'out': tf.Variable(tf.random_normal([n_classes]))
}

# Dropout parameter
keep_prob = tf.placeholder(tf.float32)

# Create model
def multilayer_perceptron(x):
    # Hidden fully connected layer with sigmoid activation
    layer_1 = tf.add(tf.matmul(x, weights['h1']), biases['b1'])
    layer_1 = tf.sigmoid(layer_1)
    layer_1 = tf.nn.dropout(layer_1, keep_prob)  # apply dropout to hidden layer
    
    # Hidden fully connected layer with sigmoid activation
    layer_2 = tf.add(tf.matmul(layer_1, weights['h2']), biases['b2'])
    layer_2 = tf.sigmoid(layer_2)
    layer_2 = tf.nn.dropout(layer_2, keep_prob)
    
    # Hidden fully connected layer with sigmoid activation
    layer_3 = tf.add(tf.matmul(layer_2, weights['h3']), biases['b3'])
    layer_3 = tf.sigmoid(layer_3)
    layer_3 = tf.nn.dropout(layer_3, keep_prob)

    # Output fully connected layer with linear activation
    out_layer = tf.matmul(layer_3, weights['out']) + biases['out']
    return out_layer

# Construct model
pred = multilayer_perceptron(X)

# Define loss and optimizer
loss_op = tf.reduce_mean(tf.square(pred-Y))
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
train_op = optimizer.minimize(loss_op)

# Initializing the variables
init = tf.global_variables_initializer()

# Set up so we can save the session later
saver = tf.train.Saver()

# And start the tf session
sess = tf.Session()
sess.run(init)

###################################################

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
    time.sleep(0.3)
    teleporter.publish(cmd_pose)
    time.sleep(0.3)   # wait (in real time) before and after jumping to avoid segfaults

def calc_reward(state, action):
    """
    Give a scalar reward based on the last action
    and the current state.
    """
    if is_crashed:
        #reset_positions()
        teleport_random()
        return -1
    elif moved_forward(action):
        return +0.01
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
    cutoff_dist = 0.5
    closest_obstacle = min(state[0])
    if (closest_obstacle < cutoff_dist):
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

def crash_callback(data):
    global is_crashed
    if data.data:
        is_crashed = True
    else:
        is_crashed = False

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

    ax1.plot(iters, coll_freq, 'r-')
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Number of Collisions", color='r')

    ax2 = ax1.twinx()
    ax2.plot(iters, cu_reward, 'b-')
    ax2.set_ylabel("Cumulative Reward", color='b')

    fig.tight_layout()
    #plt.save("collision_frequency_plot.png")
    plt.show()

def start_simulator(gui=True):
    with open('%s/.ros/log/stage_from_rl_controller.log' % home_dir, 'w') as fp:
        worldfile = "%s/worlds/extrasimple.world" % base_dir
        if gui:
            proc = subprocess.Popen(["rosrun", "stage_ros", "stageros", worldfile], stdout=fp)
        else:
            # Adding -g argument runs the simulator without the gui
            proc = subprocess.Popen(["rosrun", "stage_ros", "stageros", "-g", worldfile], stdout=fp)
    return proc

def reset_positions():
    """
    Wrapper for service call to /reset_positions. Adds a delay
    to avoid random segfaults.
    """
    rospy.sleep(2)
    reset_simulation()
    rospy.sleep(2)

def estimate_uncertainty(input_data, n_passes=10, k_prob=0.8):
    """
    Use dropout to estimate uncertainty. For a given input, 
    run through the network a bunch of times with different (Bernoulli) 
    dropout masks. High variance in the results implies high uncertainty.

    n_passes is the number of different dropout masks to use
    k_prob governs how many weights to drop out
    """

    predictions = sess.run(pred, feed_dict={X: input_data, keep_prob: k_prob})
    for i in range(n_passes - 1):
        Q_predicted = sess.run(pred, feed_dict={X: input_data, keep_prob: k_prob})
        predictions = np.vstack((predictions, Q_predicted))

    # Calculate variances, one for each element in Q_predicted (left, forward, right)
    variances = np.var(predictions, axis=0)

    return variances

def partial_fit(x_data, y_data):
    """
    Fit the network weights to the given data
    """

    assert len(x_data) == len(y_data)

    N = len(x_data)  # the number of data points we're dealing with
    print(x_data.shape, y_data.shape)

    training_epochs = 100
    display_step = 10
    batch_size = 200

    # Training cycle
    for epoch in range(training_epochs):
        avg_cost = 0.
        total_batch = int(N/batch_size)
        # Loop over all batches
        for i in range(total_batch):
            # Get next batch
            batch_x = x_data[batch_size*i:batch_size*(i+1)]
            batch_y = y_data[batch_size*i:batch_size*(i+1)]

            # Run optimization op (backprop) and cost op (to get loss value)
            _, c = sess.run([train_op, loss_op], feed_dict={X: batch_x, Y: batch_y, keep_prob: 0.9})
            # Compute average loss
            avg_cost += c / total_batch
        # Display logs per epoch step
        if (epoch + 1) % display_step == 0:
            print("Epoch:", '%04d' % (epoch+1), "cost={:.9f}".format(avg_cost))
    print("Optimization Finished!")


def main():

    # set initial command velocities to 0
    cmd_vel = Twist()
    cmd_pose = Pose()   # also initialize a pose for teleportation purposes

    update_interval = 1000  # how many actions to take before retraining
    
    X_rand = np.vstack([np.array([8*random.random() for i in range(180)]).reshape(1,-1) for i in range(update_interval)])  # random sensor input
    y_rand = np.vstack([np.array([1 for i in range(3)]).reshape(1,-1) for i in range(update_interval)])      # start with equal value on all actions

    # Train on random data initially, just so we can fit to something
    #partial_fit(X_rand,y_rand)
    
    last_action = 1
    last_state = X_rand[-1]  # take the last randomly generated entry to be the "previous state" for initialization
    old_Q = sess.run(pred, feed_dict={X: X_rand, keep_prob: 0.8})

    # initialize replay buffer
    x = X_rand  # stores states
    y = y_rand  # stores corrected Q-values values

    # variables to plot results at the end
    global iterations
    global collision_frequencies
    global cumulative_reward
    it = 1  # iteration counter

    print("==> Starting simulator")
    sim_proc = start_simulator(gui=True)   # store a process object for the simulator
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
            Q_values = sess.run(pred, feed_dict={X: state, keep_prob: 0.8})

            # estimate uncertainty using dropout
            q_variances = estimate_uncertainty(state)  # TODO: figure out how to use this
            print(q_variances)

            # Control accordingly
            action = update_twist(cmd_vel, Q_values)
            controller.publish(cmd_vel)

            # Get reward from last action
            R = calc_reward(state, last_action)

            # update things that keep track of results
            if (R == -1):
                cf += 1  # we collided, iterate the counter
            cr += R  # add the reward to our running total

            # Calculate correct Q(s,a) from last action
            # Q(s,a) = R + gamma*Q(s+1,a+1)
            corrected_Q = correct_Q(last_action, last_state, R, old_Q[0], Q_values[0])

            # Update replay buffer with correct Q(s,a)
            x = np.vstack((x, last_state))
            y = np.vstack((y, corrected_Q))

            # remember what we did this turn so we can see its result in the next step
            last_state = state
            last_action = action  
            old_Q = Q_values

            rate.sleep()

        # Drop old data from the replay buffer
        x = x[update_interval:]    
        y = y[update_interval:]

        # Update network from replay buffer
        print("==> Retraining")
        partial_fit(x,y)

        # Reset the positions
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
        crash_tracker = rospy.Subscriber('/robot_0/is_crashed', Int8, crash_callback)
        reset_simulation = rospy.ServiceProxy('reset_positions', Empty)
        rate = rospy.Rate(10) # in hz

        main()
    except rospy.ROSInterruptException:
        pass
    finally:   # Always do these things before quitting

        # save the model parameters
        save_name = "%s/tmp/RLCA_saved_model" % base_dir
        #save_name += time.strftime("%Y%m%d%H%M")  # add a unique timestamp
        saver.save(sess, save_name)
        print("\n\nSaved Parameters as %s\n\n" % save_name)

        # display plots
        display_plot(iterations, collision_frequencies, cumulative_reward) 

        # close the tf session
        sess.close()  

