import rclpy
import time
from threading import Thread

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from sofar_simple_nav_interface.srv import PathService, GripperService

class RobotLogic(Node):

    def __init__(self):
        super().__init__("robot_logic_node")

        self.idle = True

        self.path_client = self.create_client(PathService, "/navigation/path")
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('PathService not available, waiting...') 

        self.grasp_client = self.create_client(GripperService, "/robot/grasp")
        while not self.grasp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('GripperService not available, waiting...') 

        self.ack_sub = self.create_subscription(Bool, "/controller/ack", self.on_ack_received, 10)

        self.goal_pub = self.create_publisher(Point, "/controller/goal", 10)


    def on_ack_received(self, msg: Bool):
        self.get_logger().info("Got ack from controller")
        self.idle = msg.data


    def get_path_by_idx(self, idx: int):
        req = PathService.Request()
        req.path_idx.data = idx
        response = self.path_client.call(req)
        return response.path
    

    def attempt_grasp(self):
        grasp_req = GripperService.Request()
        response = self.grasp_client.call(grasp_req)
        return response.grasped.data
    

    def routine(self):
        # Get path from robot to crate
        path_to_crate = self.get_path_by_idx(0)
        # Iterate over waypoints
        for waypoint in path_to_crate[1:]:
            self.goal_pub.publish(waypoint)
            self.idle = False
            # Wait for ack from controller
            while not self.idle:
                time.sleep(1)
        # Crate reached, try grasp...
        if self.attempt_grasp() == True:
            # Get path to goal
            path_to_goal = self.get_path_by_idx(1)
            # Iterate over waypoints
            for waypoint in path_to_goal[1:]:
                self.goal_pub.publish(waypoint)
                self.idle = False
                # Wait for ack from controller
                while not self.idle:
                    time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    # Wait for other nodes to init properly
    time.sleep(1)

    # Create and spin controller node
    logic = RobotLogic()
    
    # Spinning thread to make sure callbacks are executed
    spin_thread = Thread(target=rclpy.spin, args=(logic,))
    spin_thread.start()

    # Start logic node routine
    logic.routine()

    # On shutdown..
    logic.get_logger().info("Shutdown logic node...")
    logic.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == '__main__':
    main()


