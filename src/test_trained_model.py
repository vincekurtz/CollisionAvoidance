#!/usr/bin/env python

##
# 
#  Load a trained model (generated by rl_controller.py) and
#  use it to run a simulated agent.
#
##

from rl_controller import *

def load_model(fname):
    global q_net   # defined in rl_controller.py
    q_net = joblib.load(fname)

def sensor_callback(data):
    """
    Update the data from the sensor as we get it
    """
    global sensor_data
    sensor_data = data.ranges

def main():
    # start the simulator
    sim_pid = start_simulator(gui=True)
    rate.sleep()

    load_model("/home/vince/catkin_ws/src/collision_avoidance/tmp/trained_model.pkl")
    
    # set initial velocities to 0
    cmd = Twist()

    rospy.sleep(1)

    while not rospy.is_shutdown():
        print("==> Running")
        state = np.array(sensor_data).reshape(1,-1)

        if is_crashed():
            reset_positions()
            rate.sleep()

        # move according to RL controller
        Q_values = q_net.predict(state)
        action = update_twist(cmd, Q_values)  # uses epsilon-greedy
        controller.publish(cmd)

        rate.sleep()

if __name__=="__main__":
    try:
        # Initialize ros node and publishers/subscribers
        rospy.init_node('rl_controller', anonymous=True)
        controller = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        sensor = rospy.Subscriber('/robot_0/base_scan', LaserScan, sensor_callback)
        tracker = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, odom_callback)
        reset_positions = rospy.ServiceProxy('reset_positions', Empty)
        rate = rospy.Rate(10) # in hz

        main()
    except rospy.ROSInterruptException:
        pass

