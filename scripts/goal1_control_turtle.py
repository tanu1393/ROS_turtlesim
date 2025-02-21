#!/usr/bin/env python
import rospy
import random
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64, Float32
from turtlesim.msg import Pose
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp #proportional
        self.Ki = Ki #integral
        self.Kd = Kd #derivative
        self.prev_error = 0
        self.integral = 0 
    
    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class TurtlePIDController:
    def __init__(self):

        # Publishers
        self.pub_velocity = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.init_node("goal1", anonymous=True)
        rospy.loginfo("Node started!")

        # Subscribers
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # PID Controllers
        self.angle_pid = PID(Kp=2.0, Ki=0.1, Kd=0.05)
        self.distance_pid = PID(Kp=1.5, Ki=0.05, Kd=0.02)

        self.rate = rospy.Rate(10)  # 10 Hz update rate
        
        # Turtle Pose
        self.turtle_pose = Pose()

         # Randomly select a starting position (avoid edges)
        self.start_x = random.uniform(2.0, 8.0)
        self.start_y = random.uniform(2.0, 8.0)

        # Set the goal position (different from start position)
        self.goal_x = random.uniform(1.0, 10.0)
        self.goal_y = random.uniform(1.0, 10.0)
        
        # Spawn a new turtle at the random starting position
        rospy.wait_for_service('/spawn')
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(self.start_x, self.start_y, 0, "turtle2")

        rospy.loginfo(f"Spawned turtle at ({self.start_x}, {self.start_y})")
        rospy.loginfo(f"Goal set at ({self.goal_x}, {self.goal_y})")

        # Create ROS topics to visualize with rqt_plot
        self.error_pub = rospy.Publisher('/pid/error', Float32, queue_size=10)
        self.control_input_pub = rospy.Publisher('/pid/control_input', Float32, queue_size=10)
        self.position_pub = rospy.Publisher('/pid/position', Point, queue_size=10)
        
        # Data logging for visualization
        self.error_log = []
        self.control_inputs = []
        self.positions = []
        self.time_log = []
        self.start_time = rospy.get_time()
    
    def pose_callback(self, msg):
        """Updates current turtle position and calculates error."""
        self.turtle_pose = msg
    
    def move_turtle(self):
        twist_data = Twist()
        while not rospy.is_shutdown():
            dist_err = math.sqrt((self.goal_x - self.turtle_pose.x) ** 2 + (self.goal_y - self.turtle_pose.y)**2)
            rospy.loginfo(f"Distance to goal: {dist_err}")
            
            goal_theta = math.atan2(self.goal_y - self.turtle_pose.y, self.goal_x - self.turtle_pose.x)
            angle_error = goal_theta - self.turtle_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize angle

            # Compute time difference for PID
            current_time = rospy.get_time()
            dt = current_time - self.start_time
            self.start_time = current_time

            # Compute PID outputs
            angular_correction = self.angle_pid.update(angle_error, dt)
            linear_correction = self.distance_pid.update(dist_err, dt)
            rospy.loginfo(f"PID Output: linear={linear_correction}, angular={angular_correction}")
            # Set velocity commands
            twist_data.angular.z = max(min(angular_correction, 2.0), -2.0)  # Limit rotation speed
            twist_data.linear.x = max(min(linear_correction, 2.0), 0.0) if abs(angle_error) < 0.1 else 0.0  # Move only when facing goal
            rospy.loginfo(f"Publishing cmd_vel: linear={twist_data.linear.x}, angular={twist_data.angular.z}")
            
            self.pub_velocity.publish(twist_data)

            # Publish error and control input for rqt_plot
            self.error_log.append(dist_err)
            self.control_inputs.append(linear_correction)
            self.positions.append((self.turtle_pose.x, self.turtle_pose.y))
            # Logging for visualization
            self.time_log.append(current_time)

            # Publish error, control input, and position to ROS topics
            self.error_pub.publish(Float32(dist_err))
            self.control_input_pub.publish(Float32(linear_correction))
            self.position_pub.publish(Point(self.turtle_pose.x, self.turtle_pose.y, 0))

            # Stop condition
            if dist_err < 0.1:
                print("Goal Reached!")
                break
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtlePIDController()
        controller.move_turtle()
    except rospy.ROSInterruptException:
        pass


