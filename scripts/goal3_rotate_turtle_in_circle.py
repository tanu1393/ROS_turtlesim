#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class CircularTurtle:
    def __init__(self):
        rospy.init_node("goal3_rotate_turtle_in_circle", anonymous=True)

        # Publisher
        self.pub_velocity = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.real_pose_pub = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)
        self.noisy_pose_pub = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10)

        # Subscribe to turtle pose
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Control parameters
        self.circle_speed = 1.0  # Linear speed
        self.circle_radius = 2.0  # Radius
        self.noise_std = 10  # Standard deviation for noise

        self.turtle_pose = Pose()  # Store pose
        rospy.Timer(rospy.Duration(5), self.publish_poses)
        
    def pose_callback(self, msg):
        """Update the turtle's current position."""
        self.turtle_pose = msg

    def move_in_circle(self):
        """Make the turtle move in a circular path."""
        rate = rospy.Rate(10)  # 10 Hz
        twist = Twist()
        twist.linear.x = self.circle_speed
        twist.angular.z = self.circle_speed / self.circle_radius  # ω = v / r

        while not rospy.is_shutdown():
            self.pub_velocity.publish(twist)
            self.publish_poses()

            rate.sleep()

    def publish_poses(self):
        """Publish real and noisy turtle pose."""
        # Publish real pose
        self.real_pose_pub.publish(self.turtle_pose)
        rospy.loginfo("Published Real Pose: x=%.2f, y=%.2f", self.turtle_pose.x, self.turtle_pose.y)
        # Create noisy pose
        noisy_pose = Pose()
        noisy_pose.x = self.turtle_pose.x + random.gauss(0, self.noise_std)
        noisy_pose.y = self.turtle_pose.y + random.gauss(0, self.noise_std)
        noisy_pose.theta = self.turtle_pose.theta + random.gauss(0, self.noise_std)

        self.noisy_pose_pub.publish(noisy_pose)
        rospy.loginfo("Published Noisy Pose: x=%.2f, y=%.2f", noisy_pose.x, noisy_pose.y)

if __name__ == '__main__':
    try:
        turtle = CircularTurtle()
        turtle.move_in_circle()
    except rospy.ROSInterruptException:
        pass

