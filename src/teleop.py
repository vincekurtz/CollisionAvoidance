#!/usr/bin/env python

##
#
# Teleoperate a single robot using the arrow keys and
# velocity control: that is, you can change forward speed
# and angular velociy.
#
##

import rospy
import sys,tty,termios
from geometry_msgs.msg import Twist

class _Getch:
    """
    Capture a particular keystroke.

    up: \x1b[A
    down: \x1b[B
    right: \x1b[C
    left: \x1b[D
    """
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            first = sys.stdin.read(1)
            if first=='\x03':
                # this is control-c
                raise KeyboardInterrupt
            next2 = sys.stdin.read(2)
            ch = first + next2
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def get():
    """
    Capture a keypress and print the result.
    """
    inkey = _Getch()
    while(1):
        k=inkey()
        if k!='':break
    if k=='\x1b[A':
        return "up"
    elif k=='\x1b[B':
        return "down"
    elif k=='\x1b[C':
        return "right"
    elif k=='\x1b[D':
        return "left"
    else:
        return

def update_twist(twist, key):
    """
    Capture keyboard input and use this to produce an
    updated twist message
    """
    # Parameters for changes
    d_lin = 0.1
    d_ang = 0.1

    if key == "up":
        twist.linear.x += d_lin
    elif key == "down":
        twist.linear.x -= d_lin
    elif key == "left":
        twist.angular.z += d_lin
    elif key == "right":
        twist.angular.z -= d_lin

def main():
    # Create a new ROS node with a unique name
    rospy.init_node('teleop', anonymous=True)
    controller = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10 hz

    # set initial command velocities to 0
    cmd = Twist()

    while not rospy.is_shutdown():
        key = get()
        update_twist(cmd, key)
        print cmd
        controller.publish(cmd)
        #rate.sleep()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
