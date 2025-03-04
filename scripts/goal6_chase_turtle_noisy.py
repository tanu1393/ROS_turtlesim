#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from turtlesim.srv import Spawn
import math
import threading
import numpy as np

class KalmanFilter:
    def __init__(self, speed, radius):
        self.speed = speed
        self.radius = radius
        self.dt = 0.1  # Time step
        self.x = np.array([[0.0], [0.0], [0.0]]) # Initial state (x, y, theta)
        self.P = np.eye(3) * 100 # Initial covariance
        self.F = np.array([[1, 0, -self.dt * self.speed * np.sin(0)],
                           [0, 1, self.dt * self.speed * np.cos(0)],
                           [0, 0, 1]]) # State transition matrix
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0]]) # Measurement matrix
        self.Q = np.eye(3) * 0.1 # Process noise covariance
        self.R = np.eye(2) * 10 # Measurement noise covariance

    def update(self, measurement):
        # Prediction step
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Measurement update step
        z = np.array([[measurement.x], [measurement.y]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P

        # Return estimated pose
        estimated_pose = Pose()
        estimated_pose.x = self.x[0, 0]
        estimated_pose.y = self.x[1, 0]
        estimated_pose.theta = self.x[2, 0]
        return estimated_pose
    
class SlowChaseTurtleNoisy:
    def __init__(self):
        rospy.init_node("goal6_chase_turtle_noisy", anonymous=True)

        # Publisher
        self.rt_pub_velocity = rospy.Publisher('/RobberTurtle/cmd_vel', Twist, queue_size=10)
        self.pt_pub_velocity = rospy.Publisher('/PoliceTurtle/cmd_vel', Twist, queue_size=10)
        self.noisy_pose_pub = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10) # Publish noisy pose

        # Subscriber
        rospy.Subscriber('/PoliceTurtle/pose', Pose, self.pt_pose_callback)
        rospy.Subscriber('/rt_noisy_pose', Pose, self.rt_pose_callback) # Subscribe to noisy pose

        # Control parameters
        self.circle_speed = 1.5  # Linear speed
        self.circle_radius = 5.0  # Radius
        self.chase_active = True
        self.rt_pose = Pose()  # Store rt pose
        self.pt_pose = Pose()
        self.noise_std = 1 # Standard deviation for noise
        rospy.Timer(rospy.Duration(5), self.publish_noisy_poses) # Publish noisy poses
        
        self.kalman_filter = KalmanFilter(self.circle_speed, self.circle_radius) # Initialize Kalman filter
        self.last_rt_pose_time = rospy.Time.now() #Store last pose update time
        self.predicted_rt_pose = Pose() #Store predicted pose
        
        # Spawn RT at center moving in a circle
        self.start_x = 5.5  # Center of turtlesim
        self.start_y = 5.5 - self.circle_radius
        rospy.wait_for_service('/spawn')
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(self.start_x, self.start_y, 0, "RobberTurtle")

        # Spawn PT after 10 seconds
        rospy.Timer(rospy.Duration(10), self.spawn_pt, oneshot=True)

        # Start RT movement in a separate thread
        threading.Thread(target=self.move_in_circle).start()


    def publish_noisy_poses(self, event):
        """Publish noisy turtle pose."""
        # Create noisy pose
        noisy_pose = Pose()
        noisy_pose.x = self.rt_pose.x + random.gauss(0, self.noise_std)
        noisy_pose.y = self.rt_pose.y + random.gauss(0, self.noise_std)
        noisy_pose.theta = self.rt_pose.theta + random.gauss(0, self.noise_std)

        self.noisy_pose_pub.publish(noisy_pose)
        rospy.loginfo("Published Noisy Pose: x=%.2f, y=%.2f", noisy_pose.x, noisy_pose.y)

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
        """Update RT's pose using Kalman filter and predict future pose."""
        self.predicted_rt_pose = msg #Update predicted pose with new measurement
        self.last_rt_pose_time = rospy.Time.now() #Update last pose time
        self.predicted_rt_pose = self.kalman_filter.update(msg) # Update with Kalman filter
        self.rt_pose = self.predicted_rt_pose
      
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
        # Select a point ahead of RT on its circular trajectory
        future_theta = math.atan2(ry - 5.5, rx - 5.5) + math.pi  # 180° ahead of RT

        # Move PT ahead along RT’s trajectory by a time T (e.g., 2 sec ahead)
        future_theta = future_theta + (self.circle_speed / self.circle_radius) * 1.5 
        wait_x = 5.5 + self.circle_radius * math.cos(future_theta)
        wait_y = 5.5 + self.circle_radius * math.sin(future_theta)

        return wait_x, wait_y

    def predict_rt_pose(self, current_pose, dt):
        """Predicts RT's pose after a time interval dt."""
        predicted_pose = Pose()
        predicted_pose.x = current_pose.x + self.circle_speed * math.cos(current_pose.theta) * dt
        predicted_pose.y = current_pose.y + self.circle_speed * math.sin(current_pose.theta) * dt
        predicted_pose.theta = current_pose.theta + (self.circle_speed / self.circle_radius) * dt
        return predicted_pose
    
    def chase_rt(self):
        rate = rospy.Rate(5)
        twist = Twist()
        self.last_rt_pose_time = rospy.Time.now() # Initialize last pose time
        ambush_reached = False
        predicted_rt_x, predicted_rt_y = self.compute_ambush_position()
        rospy.loginfo("calculated ambush position: x=%.2f, y=%.2f", predicted_rt_x, predicted_rt_y)
        while not rospy.is_shutdown() and self.chase_active:

            # Calculate distance to predicted position
            distance_to_prediction = math.sqrt((predicted_rt_x - self.pt_pose.x)**2 + (predicted_rt_y - self.pt_pose.y)**2)
            if not ambush_reached:
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
                    ambush_reached = True
                    rospy.loginfo("PT reached ambush position.")
                    twist = Twist()
                    self.pt_pub_velocity.publish(twist)
            else:
                time_since_last_update = (rospy.Time.now() - self.last_rt_pose_time).to_sec()
                predicted_rt_pose = self.predict_rt_pose(self.predicted_rt_pose, time_since_last_update)
                distance_to_rt = math.sqrt((predicted_rt_pose.x - self.pt_pose.x)**2 + (predicted_rt_pose.y - self.pt_pose.y)**2)
                self.predicted_rt_pose = predicted_rt_pose 
                
                if distance_to_rt < 0.5:  # RT is near, catch it
                    self.chase_active = False
                    self.pt_pub_velocity.publish(Twist())
                    self.rt_pub_velocity.publish(Twist())

                    rospy.loginfo("Chase complete!")
                    break
  
if __name__ == '__main__':
    try:
        turtle = SlowChaseTurtleNoisy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass