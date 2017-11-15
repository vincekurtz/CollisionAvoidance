#!/usr/bin/env python

##
#
# The simplest automatic controller you could imagine:
#
#   If there's something ahead, turn away from obstacles
#   Otherwise just go straight
#
##

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import random

# Sensor data stored in a global variable so it can be accessed asynchronously
sensor_data = LaserScan().ranges

def update_twist(twist, obstacles):
    """
    Given the distances of left, right, and center obstacles,
    move accordingly.
    """
    d_lin = 0.5  # step sizes
    d_ang = 0.5

    (left, right, center) = obstacles  # unpack distances

    cutoff_dist = 3.0
    if (center < cutoff_dist):
        # choose the side with further obstacles if there's something in the way
        if (left < right):
            print("turning right")
            twist.linear.x = d_lin/2  #slow down but keep going forward
            twist.angular.z = -d_ang
        elif (right < left):
            print("turning left")
            twist.linear.x = d_lin/2  #slow down but keep going forward
            twist.angular.z = d_ang

    else:
        print("going straight")
        twist.linear.x = d_lin
        twist.angular.z = 0


def sensor_callback(data):
    """
    Handle new sensor data by updating a global variable
    """
    global sensor_data
    sensor_data = data.ranges   # raw numbers for each angle increment

def calc_obstacles(ranges):
    """
    Based on the given raw sensor data, give the closest 
    distance of any obstacles on the left, the right, and straight ahead.

    Note that a distance of 8 means no obstacle could be seen
    """
    right_ranges = ranges[0:30]
    center_ranges = ranges[60:120]
    left_ranges = ranges[160:180]

    if (len(ranges) != 0):
        obstacles = (min(left_ranges), min(center_ranges), min(right_ranges))
    else:
        # we probably just initialized and haven't gotten an update yet
        obstacles = (0.0, 0.0, 0.0)

    return obstacles

def teleport_random():
    """
    Teleport the robot to a new random position
    """
    x_min = -8
    x_max = 8
    y_min = -8
    y_max = 8

    # Randomly generate a pose
    cmd_pose = Pose()
    cmd_pose.position.x = random.uniform(x_min, x_max)
    cmd_pose.position.y = random.uniform(y_min, y_max)
    cmd_pose.orientation.z = random.uniform(-7,7) # approximately 360degrees from a quaternarion
    cmd_pose.orientation.w = 1

    teleporter.publish(cmd_pose)

def main():
    # set initial command velocities to 0
    cmd = Twist()

    while not rospy.is_shutdown():
        teleport_random()  # reset to a new position every so often

        for i in range(100):
            # Sensor data updated asynchronously and stored in global var sensor_data
            # Tell if there are obstacles in front, to the left, or to the right 
            obstacles = calc_obstacles(sensor_data)

            # Control based on sensor data
            update_twist(cmd, obstacles)

            controller.publish(cmd)
            rate.sleep()

if __name__=='__main__':
    try:
        # Create new ROS node with unique name
        rospy.init_node('naive_controller', anonymous=True)
        controller = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        teleporter = rospy.Publisher('/robot_0/cmd_pose', Pose, queue_size=10)
        sensor = rospy.Subscriber('/robot_0/base_scan', LaserScan, sensor_callback)
        rate = rospy.Rate(10) # 10 hz

        main()
    except rospy.ROSInterruptException:
        pass
