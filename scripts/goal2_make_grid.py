#!/usr/bin/env python
import rospy
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
        rospy.init_node("goal2_make_grid", anonymous=True)
        rospy.loginfo("Node initialised!")

        # Subscribers
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # PID Controllers
        self.angle_pid = PID(Kp=1.0, Ki=0.0001, Kd=0.1)
        self.distance_pid = PID(Kp=1.0, Ki=0.0001, Kd=0.1)

        self.rate = rospy.Rate(10)  # 10 Hz update rate
        
        # Turtle Pose
        self.turtle_pose = Pose()
        
        # Define Grid Waypoints [(x, y, theta)]
        self.grid_waypoints = [
            (1, 1, 0), (10, 1, 90), (10, 2, 180), (1, 2, 90), 
            (1, 3, 0), (10, 3, 90), (10, 4, 180), (1, 4, 90),
            (1, 5, 0), (10, 5, 0)
        ]
        # Create ROS topics to visualize with rqt_plot
        self.error_pub = rospy.Publisher('/pid/error', Float32, queue_size=10)
        self.control_input_pub = rospy.Publisher('/pid/control_input', Float32, queue_size=10)
        self.position_pub = rospy.Publisher('/pid/position', Point, queue_size=10)
        self.vel_pub_plot = rospy.Publisher('/pid/velocity', Float32, queue_size=10)
        
        # Acceleration and Deceleration Limits
        self.max_acceleration = 0.5  # Maximum velocity increase per iteration
        self.max_deceleration = 0.6  # Maximum velocity decrease per iteration
        self.previous_velocity = 0.0
        # Data logging for visualization
        
        self.error_log = []
        self.control_inputs = []
        self.positions = []
        self.time_log = []
        self.start_time = rospy.get_time()

    def pose_callback(self, msg):
        """Updates current turtle position and calculates error."""
        self.turtle_pose = msg

    def move_turtle(self, goal_x, goal_y):
        rospy.loginfo(f"MOVE TURTLE (x, y) :={goal_x}, {goal_y}")
            
        twist_data = Twist()
        while not rospy.is_shutdown():
            dist_err = math.sqrt((goal_x - self.turtle_pose.x) ** 2 + (goal_y - self.turtle_pose.y)**2)

            # Stop condition
            if dist_err < 0.1:
                rospy.loginfo(f"SubGoal Reached!")
                break

            goal_theta = math.atan2(goal_y - self.turtle_pose.y, goal_x - self.turtle_pose.x)
            angle_error = goal_theta - self.turtle_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize angle

            # Compute time difference for PID
            current_time = rospy.get_time()
            self.dt = current_time - self.start_time
            self.start_time = current_time

            # Compute PID outputs
            angular_correction = self.angle_pid.update(angle_error, self.dt)
            linear_correction = self.distance_pid.update(dist_err, self.dt)
           
            # Compute allowed velocity change
            delta_v = linear_correction - self.previous_velocity
            if delta_v > self.max_acceleration:
                linear_correction = self.previous_velocity + self.max_acceleration
            elif delta_v < -self.max_deceleration:
                linear_correction = self.previous_velocity - self.max_deceleration
            
            # Ensure velocity is non-negative
            linear_correction = max(linear_correction, 0.0)

            self.previous_velocity = twist_data.linear.x

            # Set velocity commands
            twist_data.angular.z = max(min(angular_correction, 2.0), -2.0)  # Limit rotation speed
            twist_data.linear.x = linear_correction if abs(angle_error) < 0.1 else 0.0
            
            self.pub_velocity.publish(twist_data)
            self.vel_pub_plot.publish(Float32(twist_data.linear.x))

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
            
            self.rate.sleep()
            
    def rotate(self, theta):
        """Rotates the turtle to a specific angle."""
        twist_data = Twist()
        while not rospy.is_shutdown():
            angle_error = theta - self.turtle_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  
            if abs(angle_error) < 0.05:
                break
            # Compute PID correction
            angular_correction = self.angle_pid.update(angle_error, self.dt)

            # Set rotation speed
            twist_data.angular.z = max(min(angular_correction, 2.0), -2.0)

            self.pub_velocity.publish(twist_data)

            self.rate.sleep()

    def move_turtle_grid(self):
        """Moves the turtle in a grid pattern."""
        for x, y, theta in self.grid_waypoints:
            self.move_turtle(x, y)
            
            # Stop completely before rotating
            stop_msg = Twist()
            self.pub_velocity.publish(stop_msg)
            rospy.sleep(0.5)  # Give some time to stop

            # Rotate to desired heading
            self.rotate(math.radians(theta))  # Ensure correct theta input

if __name__ == '__main__':
    try:   
        controller = TurtlePIDController()
        controller.move_turtle_grid()
    except rospy.ROSInterruptException:
        pass


