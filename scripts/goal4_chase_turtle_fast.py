#!/usr/bin/env python
import rospy
import random
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import threading

class ChaseTurtle:
    def __init__(self):
        rospy.init_node("goal4_chase_turtle_fast", anonymous=True)

        # Publisher
        self.rt_pub_velocity = rospy.Publisher('/RobberTurtle/cmd_vel', Twist, queue_size=10)
        self.pt_pub_velocity = rospy.Publisher('/PoliceTurtle/cmd_vel', Twist, queue_size=10)
        self.real_pose_pub = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)

        # Define a reasonable circle radius for turtlesim (scaled down)
        self.circle_radius = 5.0  # Fits within the 11x11 window
        self.circle_speed = 1.5  # Speed for the circular motion
        self.pt_base_speed = self.circle_speed * 1.8  # PT should always be faster than RT
        self.chase_active = True
        self.max_speed = self.circle_speed * 3.5  # PT max speed is much higher

        # Spawn  the turtle at a position that allows full movement
        self.start_x = 5.5  # Center horizontally
        self.start_y = 5.5 - self.circle_radius  # Offset downward to create the circle

        # Spawn a new turtle at the random starting position
        rospy.wait_for_service('/spawn')
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(self.start_x, self.start_y, 0, "RobberTurtle")
        
        # Subscribe to turtle pose
        rospy.Subscriber('/RobberTurtle/pose', Pose, self.rt_pose_callback)

        self.rt_pose = Pose()
        self.pt_pose = Pose()
        rospy.Timer(rospy.Duration(5), self.publish_poses)
        
         # Police Turtle properties
        self.acceleration = 0.2  # Speed increment per cycle
        self.deceleration = 0.3  # Speed reduction per cycle
        self.pt_current_speed = 0.0  # Track PT's speed

        # Spawn PT after 10 sec
        rospy.Timer(rospy.Duration(10), self.spawn_pt, oneshot=True)

        # Start RT movement in a separate thread
        threading.Thread(target=self.move_in_circle).start()

    def spawn_pt(self, event):
        """Spawn PT 10 seconds after RT has started moving."""
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        self.pt_x = random.uniform(2.0, 8.0)
        self.pt_y = random.uniform(2.0, 8.0)
        
        spawn_turtle(self.pt_x, self.pt_y, 0, "PoliceTurtle")
        
        # Subscribe to PT's own pose
        rospy.Subscriber('/PoliceTurtle/pose', Pose, self.pt_pose_callback)
        rospy.Subscriber('/rt_real_pose', Pose, self.rt_pose_callback)

        threading.Thread(target=self.chase_rt).start()
    
    def rt_pose_callback(self, msg):
        """Update RT's pose."""
        self.rt_pose = msg

    def pt_pose_callback(self, msg):
        """Update PT's pose."""
        self.pt_pose = msg

    def publish_poses(self):
        """Publish real and noisy turtle pose."""
        self.real_pose_pub.publish(self.rt_pose)

    def move_in_circle(self):
        """Make the turtle move in a circular path."""
        rate = rospy.Rate(10)  # 10 Hz
        twist = Twist()
        twist.linear.x = self.circle_speed
        twist.angular.z = self.circle_speed / self.circle_radius  # ω 
        rospy.sleep(2)  # Give some time for spawning

        while not rospy.is_shutdown() and self.chase_active:
            self.rt_pub_velocity.publish(twist)
            self.publish_poses()
            rate.sleep()

    def chase_rt(self):
        """Move PT towards RT using the latest received pose."""
        rate = rospy.Rate(5)
        twist = Twist()

        while not rospy.is_shutdown() and self.chase_active:
            distance = math.sqrt((self.rt_pose.x - self.pt_pose.x) ** 2 + (self.rt_pose.y - self.pt_pose.y) ** 2)

            if distance > 0.5:  # Keep chasing if distance > 0.5 units
                # Compute target angle
                goal_theta = math.atan2(self.rt_pose.y - self.pt_pose.y, self.rt_pose.x - self.pt_pose.x)
                angle_error = goal_theta - self.pt_pose.theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize angle

                # Adjust speed smoothly
                target_speed = max(self.circle_speed * 2.0, min(self.max_speed, distance * 1.5))

                if target_speed > self.pt_current_speed:
                    self.pt_current_speed = min(target_speed, self.pt_current_speed + self.acceleration)
                else:
                    self.pt_current_speed = max(target_speed, self.pt_current_speed - self.deceleration)

                # Update velocity
                twist.linear.x = self.pt_current_speed * max(0.1, 1 - abs(angle_error))

                twist.angular.z = max(min(angle_error * 1.5, 1.5), -1.5)  # Adjust turning speed
                
                self.pt_pub_velocity.publish(twist)

            else:
                self.chase_active = False
                self.pt_pub_velocity.publish(Twist())
                self.rt_pub_velocity.publish(Twist())
                rospy.loginfo("Chase complete!")
                break

if __name__ == '__main__':
    try:
        turtle = ChaseTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

