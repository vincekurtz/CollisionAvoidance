#!/usr/bin/env python

##
# 
#  Load a trained model (generated by rl_controller.py) and
#  use it to run a simulated agent.
#
##

base_dir = "/home/vjkurtz/catkin_ws/src/collision_avoidance"

from rl_controller import *

def sensor_callback(data):
    """
    Update the data from the sensor as we get it
    """
    global sensor_data
    sensor_data = data.ranges

def crash_callback(data):
    global is_crashed
    if data.data:
        is_crashed = True
    else:
        is_crashed = False

def main():
    # start the simulator
    sim_pid = start_simulator(gui=True)

    # set initial velocities to 0
    cmd = Twist()

    rospy.sleep(1)

    print("==> Running")
    with tf.Session() as sess:
        # Load saved session
        new_saver = tf.train.import_meta_graph('%s/tmp/RLCA_saved_model.meta' % base_dir)
        new_saver.restore(sess, tf.train.latest_checkpoint('./'))
        
        while not rospy.is_shutdown():
            state = np.array(sensor_data).reshape(1,-1)

            if is_crashed:
                #teleport_random()
                reset_positions()
                rate.sleep()

            # move according to RL controller
            Q_values = sess.run(pred, feed_dict={X: state})
            action = update_twist(cmd, Q_values)  # uses epsilon-greedy
            controller.publish(cmd)

            rate.sleep()

if __name__=="__main__":
    try:
        # Initialize ros node and publishers/subscribers
        rospy.init_node('rl_controller', anonymous=True)
        controller = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        teleporter = rospy.Publisher('/robot0/cmd_pose', Pose, queue_size=10)
        sensor = rospy.Subscriber('/robot_0/base_scan', LaserScan, sensor_callback)
        crash_tracker = rospy.Subscriber('/robot_0/is_crashed', Int8, crash_callback)
        reset_positions = rospy.ServiceProxy('reset_positions', Empty)
        rate = rospy.Rate(100) # in hz

        main()
    except rospy.ROSInterruptException:
        pass

