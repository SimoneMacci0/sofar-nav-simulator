import rclpy
import math
import numpy as np

from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose2D, Point, Twist

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        self.current_pose = Pose2D()
        self.dt = 0.05

        # Define PID controllers for linear velocity and rotation
        self.lin_pid = PID(5.0, 0.001, 0.2)
        self.ang_pid = PID(30.0, 2, 7.5)
        
        # Controllers update frequency (equivalent to publisher rate)
        self.lin_pid.sample_time = self.dt
        self.ang_pid.sample_time = self.dt

        # Position and angular errors threshold and control saturation value
        self.lin_threshold = 1.0
        self.ang_threshold = 0.01
        self.lin_control_clip_value = 3.0
        
        # Robot pose subscriber
        self.pose_sub = self.create_subscription(Pose2D, "/robot/pose", self.pose_callback, 10)
        # Subscriber for next robot pose to reach
        self.target_pose_sub = self.create_subscription(Point, "/controller/goal", self.set_new_target_pose, 10)
        # Cmd velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        # Ack publisher
        self.ack_pub = self.create_publisher(Bool, "/controller/ack", 10)

        self.get_logger().info("Controller module initialized!")
        self.get_logger().info("Ready to received target setpoints...")

    # Pose callback to keep track of robot's current position
    def pose_callback(self, msg: Pose2D):
        self.current_pose = msg

    # Reset PID target points and start new control loop timer
    def set_new_target_pose(self, msg: Point):
        self.get_logger().info('Received new target position (x: {0}, y: {1})'.format(msg.x, msg.y))

        # Reset PID internals to clear previous errors
        self.lin_pid.reset()
        self.ang_pid.reset()

        # Set new PID setpoints for position and orientation
        self.direction = np.argmax([
            abs(msg.x - self.current_pose.x),
            abs(msg.y - self.current_pose.y)
            ])
        self.lin_pid.setpoint = msg.x if self.direction == 0 else msg.y
        self.ang_pid.setpoint = math.atan2(msg.y - self.current_pose.y, msg.x - self.current_pose.x)

        # Start timer for control loop callback
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

    # Control loop cycle callback
    def control_loop_callback(self):

        # Compute remaining errors
        ang_error = abs(self.current_pose.theta - self.ang_pid.setpoint)
        current = self.current_pose.x if self.direction == 0 else self.current_pose.y
        lin_error = abs(current - self.lin_pid.setpoint)
        #self.get_logger().info("Remaining error: (distance: {0}, rotation: {1})".format(lin_error, ang_error))  

        # If pose reached, cancel timer
        if lin_error < self.lin_threshold and ang_error < self.ang_threshold:
            self.get_logger().info('Waypoint reached')
            self.timer.cancel()
            # Publish ack message to navigator
            ack_msg = Bool()
            ack_msg.data = True
            self.ack_pub.publish(ack_msg)

        # Only rotation aligned, perform linear motion...
        elif ang_error < self.ang_threshold:
            # Compute speed control based on remaining distance to goal
            lin_control = abs(self.lin_pid(current) * self.dt)
            # Saturate speed to prevent robot from accelerating eccessively
            if lin_control >= self.lin_control_clip_value:
                lin_control = self.lin_control_clip_value
            # Publish command velocity message
            cmd_vel = Twist()
            cmd_vel.linear.x = lin_control
            self.cmd_vel_pub.publish(cmd_vel)
                
        # Perform rotation first, in order to align with new direction
        else:
            # Compute rotation control based on target rotation
            ang_control = self.ang_pid(self.current_pose.theta) * self.dt
            # Publish command velocity message
            cmd_vel = Twist()
            cmd_vel.angular.z = ang_control
            self.cmd_vel_pub.publish(cmd_vel)

 
def main(args=None):
    rclpy.init(args=args)

    # Create and spin controller node
    controller_node = RobotController()
    rclpy.spin(controller_node)

    # On shutdown..
    controller_node.get_logger().info("Shutdown controller node...")
    controller_node.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == '__main__':
    main()


