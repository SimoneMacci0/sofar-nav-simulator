from .lib.simulation import NavigationSim
import rclpy
import math
import time

from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose2D, Twist, Point

from sofar_simple_nav_interface.srv import PathService, GripperService

from ament_index_python.packages import get_package_share_directory

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 500
SCREEN_TITLE = "Nav Simulation"

TILE_SIZE = 100
OCCUPANCY_RATIO = 0.4

class NavigationSimNode(Node):

    def __init__(self):
        super().__init__("navigation_sim_node")

        # Navigation simulator
        self.sim = NavigationSim(
            SCREEN_WIDTH, 
            SCREEN_HEIGHT, 
            SCREEN_TITLE, 
            TILE_SIZE,
            OCCUPANCY_RATIO,
            get_package_share_directory("sofar_simple_nav")
        )
        self.sim.setup()
        self.sim.thread.start()

        # Robot pose publisher
        self.pose_pub = self.create_publisher(Pose2D, "/robot/pose", 10)
        self.pose_pub_timer = self.create_timer(0.0333, self.on_timer_elapsed)

        # Cmd vel subscriber
        self.create_subscription(Twist, "/robot/cmd_vel", self.on_twist_cmd, 10)

        # Gripper command subscriber
        self.create_service(GripperService, "/robot/grasp", self.on_gripper_service_request)

        # Path service to retrieve waypoints
        self.create_service(PathService, "/navigation/path", self.on_path_service_request)

    # Timer callback for publishing robot's pose
    def on_timer_elapsed(self):
        x, y, theta = self.sim.get_robot_pose()
        pose_msg = Pose2D()
        pose_msg.x = float(x)
        pose_msg.y = float(y)
        pose_msg.theta = math.radians(theta)
        self.pose_pub.publish(pose_msg)

    # Twist msg callback
    def on_twist_cmd(self, msg: Twist):
        self.sim.update_robot_pose(msg.linear.x, msg.angular.z)

    # Callback invoked whenever robot tries to grasp crate
    def on_gripper_service_request(self, request: GripperService.Request, response: GripperService.Response):
        if self.sim.try_pick_crate():
            response.grasped.data = True
        else:
            self.get_logger().info("Nothing to grasp...")
            response.grasped.data = False
        return response

    # Callback for retrieving path waypoints 
    def on_path_service_request(self, request: PathService.Request, response: PathService.Response):
        self.get_logger().info("Received request for path with idx: {0}".format(request.path_idx.data))
        if request.path_idx.data == 0:
            for node in self.sim.path_to_crate_int:
                point_msg = Point()
                px,py = self.sim.get_tile_center_coordinates(node[0], node[1])
                point_msg.x = float(px)
                point_msg.y = float(py)
                response.path.append(point_msg)
            return response
        
        elif request.path_idx.data == 1:
            for node in self.sim.path_to_goal_int:
                point_msg = Point()
                px,py = self.sim.get_tile_center_coordinates(node[0], node[1])
                point_msg.x = float(px)
                point_msg.y = float(py)
                response.path.append(point_msg)
            return response


def main(args=None):
    
    rclpy.init(args=args)
    sim_node = NavigationSimNode()

    print("Press Ctrl+C to exit...")
    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()
