#!/usr/bin/env python
import rospy
import random
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import threading

class SlowChaseTurtle:
    def __init__(self):
        rospy.init_node("goal5_chase_turtle_slow", anonymous=True)

        # Publisher
        self.rt_pub_velocity = rospy.Publisher('/RobberTurtle/cmd_vel', Twist, queue_size=10)
        self.pt_pub_velocity = rospy.Publisher('/PoliceTurtle/cmd_vel', Twist, queue_size=10)
        self.real_pose_pub = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)

        # Subscriber
        rospy.Subscriber('/PoliceTurtle/pose', Pose, self.pt_pose_callback)
        rospy.Subscriber('/RobberTurtle/pose', Pose, self.rt_pose_callback)
        rospy.Subscriber('/rt_real_pose', Pose, self.rt_pose_callback)

        # Define RT's movement parameters
        self.circle_radius = 5.0  
        self.circle_speed = 1.5  # RT's speed
        self.rt_pose = Pose()
        self.pt_pose = Pose()

        # Define PT's movement parameters
        self.pt_speed = self.circle_speed / 2  # PT moves at half RT's speed
        self.chase_active = True

        # Spawn RT at center moving in a circle
        self.start_x = 5.5  # Center of turtlesim
        self.start_y = 5.5 - self.circle_radius  # Offset downward
        rospy.wait_for_service('/spawn')
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(self.start_x, self.start_y, 0, "RobberTurtle")

        # Spawn PT after 10 seconds
        rospy.Timer(rospy.Duration(10), self.spawn_pt, oneshot=True)

        # Start RT movement in a separate thread
        threading.Thread(target=self.move_rt_in_circle).start()

    def spawn_pt(self, event):
        """Spawns PT 10 seconds after RT starts moving."""
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        self.pt_x = random.uniform(2.0, 8.0)
        self.pt_y = random.uniform(2.0, 8.0)
        spawn_turtle(self.pt_x, self.pt_y, 0, "PoliceTurtle")
        threading.Thread(target=self.wait_and_attack).start()

    def rt_pose_callback(self, msg):
        """Update RT's pose."""
        self.rt_pose = msg

    def pt_pose_callback(self, msg):
        """Update PT's pose."""
        self.pt_pose = msg

    def publish_poses(self):
        """Publish real and noisy turtle pose."""
        # Publish real pose
        self.real_pose_pub.publish(self.rt_pose)

    def move_rt_in_circle(self):
        """Moves RT in a circular path continuously."""
        rate = rospy.Rate(10)  # 10 Hz
        twist = Twist()
        twist.linear.x = self.circle_speed
        twist.angular.z = self.circle_speed / self.circle_radius  # Circular motion
        rospy.sleep(2)  # Give some time for spawning

        while not rospy.is_shutdown() and self.chase_active:
            self.rt_pub_velocity.publish(twist)
            self.publish_poses()
            rate.sleep()
    
    def wait_and_attack(self):
        """PT waits in an ambush position and only moves when RT is near."""
        rate = rospy.Rate(10)  # 10 Hz update rate

        # Compute best ambush position
        wait_x, wait_y = self.compute_ambush_position()

        # Move PT to the waiting position
        self.move_to_position(wait_x, wait_y)

        rospy.loginfo("PT is waiting at ambush position...")

        while not rospy.is_shutdown() and self.chase_active:
            # Compute distance to RT
            distance = math.sqrt((self.rt_pose.x - self.pt_pose.x) ** 2 + (self.rt_pose.y - self.pt_pose.y) ** 2)

            # Only move if RT is within 3 units
            if distance <= 1.5:
                rospy.loginfo("RT is close! Attacking now!")
                self.attack_rt()
                break

            rate.sleep()

    def compute_ambush_position(self):
        """Finds a strategic position where PT can wait for RT."""
        rx, ry = self.rt_pose.x, self.rt_pose.y  # RT's current position

        # Select a point ahead of RT on its circular trajectory
        future_theta = math.atan2(ry - 5.5, rx - 5.5) + math.pi / 2  # 90° ahead of RT

        # Move PT ahead along RT’s trajectory by a time T (e.g., 2 sec ahead)
        future_theta = future_theta + (self.circle_speed / self.circle_radius) * 2 
        wait_x = 5.5 + self.circle_radius * math.cos(future_theta)
        wait_y = 5.5 + self.circle_radius * math.sin(future_theta)

        return wait_x, wait_y

    def move_to_position(self, x, y):
        """Moves PT to a specific position and stops there."""
        rate = rospy.Rate(10)
        twist = Twist()
        timeout = rospy.get_time() + 5  # Allow max 5 seconds to reach
        while not rospy.is_shutdown():
            distance = math.sqrt((x - self.pt_pose.x) ** 2 + (y - self.pt_pose.y) ** 2)

            # Move towards the ambush position
            twist.linear.x = self.pt_speed * 1.0 if distance > 0.5 else 0.0
            twist.angular.z = math.atan2(y - self.pt_pose.y, x - self.pt_pose.x) - self.pt_pose.theta

            self.pt_pub_velocity.publish(twist)

            if distance <= 0.5:
                self.pt_pub_velocity.publish(Twist())  # Stop PT
                break

            rate.sleep()

    def attack_rt(self):
        """PT moves directly towards RT once it is within range."""
        rate = rospy.Rate(10)
        twist = Twist()

        while not rospy.is_shutdown():
            distance = math.sqrt((self.rt_pose.x - self.pt_pose.x) ** 2 + (self.rt_pose.y - self.pt_pose.y) ** 2)

            if distance <= 0.5:
                rospy.loginfo("Caught RT!")
                self.chase_active = False
                self.pt_pub_velocity.publish(Twist())  # Stop PT
                self.rt_pub_velocity.publish(Twist())  # Stop RT
                break
            # Compute target attack angle
            attack_angle = math.atan2(self.rt_pose.y - self.pt_pose.y, self.rt_pose.x - self.pt_pose.x)
            angle_error = math.atan2(math.sin(attack_angle - self.pt_pose.theta), math.cos(attack_angle - self.pt_pose.theta))
            # If PT is not aligned, turn first
            if not aligned:
                twist.linear.x = 0  # Stop moving forward while turning
                twist.angular.z = max(min(angle_error * 2.0, 1.0), -1.0)  # Smooth turning
                if abs(angle_error) < 0.1:  # If nearly aligned, start moving forward
                    aligned = True
            else:
                twist.angular.z = 0  # Stop turning once aligned
                twist.linear.x = self.circle_speed / 2  # Move at exactly half RT’s speed

            self.pt_pub_velocity.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        turtle = SlowChaseTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
