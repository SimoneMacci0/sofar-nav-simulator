import arcade
from threading import Thread
import math
import numpy as np
from .map import Map

# Thread to run simulation in background
class SimThread(Thread):
   def __init__(self):
      Thread.__init__(self)
   
   def run(self):
      arcade.run()

# Main class representing simulation environment
class NavigationSim(arcade.Window):

    def __init__(self, width, height, title, tile_size, ratio, resources_path):
        super().__init__(width, height, title)

        self.screen_width = width
        self.screen_height = height
        self.tile_size = tile_size
        self.resources_path = resources_path

        self.map = Map(resources_path, ratio)
        self.thread = SimThread()

        # Robot, goal and path resources
        self.robot = arcade.Sprite(":resources:images/topdown_tanks/tankBody_red_outline.png", scale=self.tile_size/50)
        self.crate = arcade.Sprite(":resources:images/tiles/boxCrate_double.png", scale=0.25*self.tile_size/50)
        self.goal = arcade.Sprite(":resources:images/items/flagGreen2.png", scale=0.25*self.tile_size/50)
        self.path_to_crate_list = arcade.SpriteList(use_spatial_hash=True)
        self.path_to_goal_list = arcade.SpriteList(use_spatial_hash=True)

        # Variable
        self.all_set = False

    def setup(self):
        self.path_to_crate_full = []
        self.path_to_goal_full = []
        self.path_to_crate_int = []
        self.path_to_goal_int = []

        # Iterate until feasible map is generated...
        while len(self.path_to_crate_int) == 0 or len(self.path_to_goal_int) == 0:
            self.map.clear()
            self.map.generate(self.screen_width, self.screen_height, self.tile_size)
            self.try_find_optimal_path()

        self.set_sprites_at_location()
        self.all_set = True
        self.crate_picked = False

    # Method to set various sprites at their map location
    def set_sprites_at_location(self):
        self.set_robot_starting_position(0,0)
        self.set_crate_position(self.map.width - 1, self.map.height - 1)
        self.set_goal_position(self.map.width - 1, 0)
        for node in self.path_to_crate_full[1:-1]:
            waypoint = arcade.Sprite(":resources:images/items/star.png", scale=0.25*self.tile_size/50)
            waypoint.center_x = (node[0] + 0.5) * self.tile_size
            waypoint.center_y = (node[1] + 0.5) * self.tile_size
            self.path_to_crate_list.append(waypoint)
        for node in self.path_to_goal_full[1:-1]:
            waypoint = arcade.Sprite(":resources:images/items/star.png", scale=0.25*self.tile_size/50)
            waypoint.center_x = (node[0] + 0.5) * self.tile_size
            waypoint.center_y = (node[1] + 0.5) * self.tile_size
            self.path_to_goal_list.append(waypoint)

    def on_draw(self):
        if self.all_set:
            self.map.render()
            self.robot.draw()
            self.goal.draw()
            if not self.crate_picked:
                self.crate.draw()
            self.path_to_crate_list.draw()
            self.path_to_goal_list.draw()

    # Method to set initial robot position in arcade's map, based on user selection
    def set_robot_starting_position(self, x, y):
        self.robot.center_x = (x + 0.5) * self.tile_size
        self.robot.center_y = (y + 0.5) * self.tile_size

    # Getter for robot's pose
    def get_robot_pose(self):
        return self.robot.center_x, self.robot.center_y, (self.robot.angle + 90)

    # Method to update robot's pose from control twist, using robot's kinematics
    def update_robot_pose(self, speed: float, ang_change: float):
        # Convert angle in degrees to radians.
        angle_rad = math.radians(self.robot.angle)
        # Rotate robot
        self.robot.angle += ang_change
        # Use math to find position change based on speed and angle
        self.robot.center_x += -speed * math.sin(angle_rad)
        self.robot.center_y += speed * math.cos(angle_rad)

    # Method to set goal position for rendering in arcade's map
    def set_goal_position(self, x, y):
        self.goal.center_x = (x + 0.5) * self.tile_size
        self.goal.center_y = (y + 0.5) * self.tile_size

    # Method to set crate's position on map
    def set_crate_position(self, x, y):
        self.crate.center_x = (x + 0.5) * self.tile_size
        self.crate.center_y = (y + 0.5) * self.tile_size

    # Method to retrieve the center coordinates of a map tile given its index (x,y) in grid map
    def get_tile_center_coordinates(self, x, y):
        return ((x + 0.5) * self.tile_size, (y + 0.5) * self.tile_size)
    
    # Try to find optimal path between start and goal
    def try_find_optimal_path(self):
        self.path_to_crate_full.clear()
        self.path_to_goal_full.clear()
        self.path_to_crate_int.clear()
        self.path_to_goal_int.clear()

        start = (0,0)
        crate = (self.map.width - 1, self.map.height - 1)
        goal = (self.map.width - 1, 0)

        self.path_to_crate_full, self.path_to_crate_int = self.map.compute_path(start, crate, self.map)
        self.path_to_goal_full, self.path_to_goal_int = self.map.compute_path(crate, goal, self.map)

    # Try to pick crate up
    def try_pick_crate(self):
        robot = np.array([self.robot.center_x, self.robot.center_y])
        crate = np.array([self.crate.center_x, self.crate.center_y])
        prev_angle = self.robot.angle
        if np.linalg.norm(robot - crate) < 10:
            print(np.linalg.norm(robot - crate))
            self.robot = arcade.Sprite(self.resources_path + "/resource/robot_with_crate.png", scale=self.tile_size/50)
            self.robot.center_x = robot[0]
            self.robot.center_y = robot[1]
            self.robot.angle = prev_angle
            self.crate_picked = True
            return True
        else:
            return False

        