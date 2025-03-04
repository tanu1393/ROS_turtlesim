#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from turtlesim.srv import Spawn
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

        # Control parameters
        self.circle_speed = 1.5  # Linear speed
        self.circle_radius = 5.0  # Radius
        self.chase_active = True
        self.rt_pose = Pose()  # Store rt pose
        self.pt_pose = Pose()
        rospy.Timer(rospy.Duration(5), self.publish_poses)

        # Spawn RT at center moving in a circle
        self.start_x = 5.5  # Center of turtlesim
        self.start_y = 5.5 - self.circle_radius  # Offset downward
        rospy.wait_for_service('/spawn')
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(self.start_x, self.start_y, 0, "RobberTurtle")
        # Spawn PT after 10 seconds
        rospy.Timer(rospy.Duration(10), self.spawn_pt, oneshot=True)

        # Start RT movement in a separate thread
        threading.Thread(target=self.move_in_circle).start()
    
    def spawn_pt(self, event):
        rospy.wait_for_service('/spawn')
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(random.uniform(1, 10), random.uniform(1, 10), 0, "PoliceTurtle")
        rospy.Subscriber('/PoliceTurtle/pose', Pose, self.pt_pose_callback)
        threading.Thread(target=self.chase_rt).start()
    
    def pt_pose_callback(self, msg):
        """Update PT's pose."""
        self.pt_pose = msg

    def rt_pose_callback(self, msg):
        """Update RT's pose."""
        self.rt_pose = msg

    def move_in_circle(self):
        """Make the turtle move in a circular path."""
        rate = rospy.Rate(10)  # 10 Hz
        twist = Twist()
        twist.linear.x = self.circle_speed
        twist.angular.z = self.circle_speed / self.circle_radius  # ω = v / r

        while not rospy.is_shutdown() and self.chase_active:
            self.rt_pub_velocity.publish(twist)
            rate.sleep()

    def compute_ambush_position(self):
        """Finds a strategic position where PT can wait for RT."""
        rx, ry = self.rt_pose.x, self.rt_pose.y  # RT's current position
        px, py = self.pt_pose.x, self.pt_pose.y # PT's current position
        # Select a point ahead of RT on its circular trajectory
        future_theta = math.atan2(ry - 5.5, rx - 5.5) + math.pi  # 180° ahead of RT

        # Move PT ahead along RT’s trajectory by a time T (e.g., 2 sec ahead)
        future_theta = future_theta + (self.circle_speed / self.circle_radius) * 1.5 
        wait_x = 5.5 + self.circle_radius * math.cos(future_theta)
        wait_y = 5.5 + self.circle_radius * math.sin(future_theta)

        return wait_x, wait_y

    def chase_rt(self):
        rate = rospy.Rate(5)
        twist = Twist()
        arrived_at_prediction = False
        predicted_rt_x, predicted_rt_y = self.compute_ambush_position()
        rospy.loginfo("calculated ambush position: x=%.2f, y=%.2f", predicted_rt_x, predicted_rt_y)
        while not rospy.is_shutdown() and self.chase_active:
            # Calculate distance to predicted position
            distance_to_prediction = math.sqrt((predicted_rt_x - self.pt_pose.x)**2 + (predicted_rt_y - self.pt_pose.y)**2)
            if not arrived_at_prediction: # Move towards predicted position if not arrived yet
                if distance_to_prediction > 0.3:  # Move towards predicted position
                    goal_theta = math.atan2(predicted_rt_y - self.pt_pose.y, predicted_rt_x - self.pt_pose.x)
                    angle_error = goal_theta - self.pt_pose.theta
                    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

                    # Respect the half-speed constraint
                    target_speed = min(self.circle_speed / 2, distance_to_prediction * 0.75)

                    twist.linear.x = target_speed * max(0.1, 1 - abs(angle_error))
                    twist.angular.z = max(min(angle_error * 1.5, 1.5), -1.5)
                    self.pt_pub_velocity.publish(twist)
                else:
                    arrived_at_prediction = True # Set flag when arrived
                    rospy.loginfo("PT arrived at predicted position.")
                    twist = Twist() # Stop moving
                    self.pt_pub_velocity.publish(twist)
                    # break
            else:  # Close enough to predicted position, wait for RT
                distance_to_rt = math.sqrt((self.rt_pose.x - self.pt_pose.x)**2 + (self.rt_pose.y - self.pt_pose.y)**2)
                if distance_to_rt < 0.5:  # RT is near, catch it
                    self.chase_active = False
                    self.pt_pub_velocity.publish(Twist())
                    self.rt_pub_velocity.publish(Twist())

                    rospy.loginfo("Chase complete!")
                    break

    def publish_poses(self, event):
        """Publish real and noisy turtle pose."""
        # Publish real pose
        self.real_pose_pub.publish(self.rt_pose)
        rospy.loginfo("Published Real Pose: x=%.2f, y=%.2f", self.rt_pose.x, self.rt_pose.y)
        
if __name__ == '__main__':
    try:
        turtle = SlowChaseTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

