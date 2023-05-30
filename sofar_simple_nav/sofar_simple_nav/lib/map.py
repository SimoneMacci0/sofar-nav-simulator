import random
import time
from .cell import Cell

CELL_TYPES = [
    "/resource/empty.png",
    "/resource/occupied.png"
]

# Map class to represent the 2D cell-world
class Map():

    def __init__(self, package_path, ratio):
        self.cells = []
        self.width = 0
        self.height = 0
        self.cell_size = 0
        self.package_path = package_path
        self.ratio = ratio
        # Initialize solver algorithm
        self.solver = Astar()       

    def get_cell(self, x, y) -> Cell:
        return self.cells[x][y]
    
    # Set neighbours of the passed cell, based on positioning on the map
    def set_cell_neighbours(self, cell: Cell):
        if cell.x < self.width - 1:
            cell.neighbours.append(self.get_cell(cell.x+1, cell.y))
        if cell.x > 0:
            cell.neighbours.append(self.get_cell(cell.x-1, cell.y))
        if cell.y < self.height - 1:
            cell.neighbours.append(self.get_cell(cell.x, cell.y+1))
        if cell.y > 0: 
            cell.neighbours.append(self.get_cell(cell.x, cell.y-1))

    # Clear previously generated map
    def clear(self):
        self.cells = []
        self.width = 0
        self.height = 0
        self.cell_size = 0

    # Method to generate map with given occupation ratio
    # @Params:
    #   - screen Width 
    #   - Screen Height
    #   - cell Size 
    def generate(self, scr_width, scr_height, cell_size):
        # Compute map width and height based on cell size and screen dimensions
        self.width = int(scr_width / cell_size)
        self.height = int(scr_height / cell_size)
        self.cell_size = cell_size
        # Cycle over each row to generate cells
        random.seed(time.time())
        for x in range(self.width):
            self.cells.append([0] * self.height)
            for y in range(self.height):
                if x == 0 and y == 0:
                    type = 0
                elif x == self.width - 1 and y == self.height - 1:
                    type = 0
                else:
                    type = 1 if random.random() < self.ratio else 0
                resource_path = self.package_path + CELL_TYPES[type]
                self.cells[x][y] = Cell(x, y, type, resource_path, cell_size)
        # Set neighbours for each cell (for planner)
        for x in range(self.width):
            for y in range(self.height):
                self.set_cell_neighbours(self.get_cell(x,y))

    # Method to draw the map on the arcade window
    def render(self):
        for x in range(self.width):
            for y in range(self.height):
                cell = self.get_cell(x,y)
                cell.center_x = (x + 0.5) * self.cell_size
                cell.center_y = (y + 0.5) * self.cell_size
                cell.draw()


    # Interpolate path to avoid start/stop between consecutive waypoints
    def interpolate(self, path):
        interpolated = []
        interpolated.append(path[0])
        idx = 1

        while idx < len(path):
            for p in path[idx:]:
                if interpolated[-1][0] == p[0] or interpolated[-1][1] == p[1]:
                    idx += 1
                else:
                    interpolated.append(path[idx-1])
                    idx += 1
                
        interpolated.append(path[-1])   
        return interpolated

    # Find optimal path from start to goal
    def compute_path(self, start, goal, map):
        self.solver.reset(map)
        path = self.solver.compute_path(start, goal)
        if len(path) > 0:
            interpolated_path = self.interpolate(path)
            return path, interpolated_path
        else:
            return [], []

    

# A* solver algorithm
class Astar:

    def __init__(self):
        # Map, starting and goal position variables
        self.start = None  # List of coordinates [start_x, start_y]
        self.goal = None # List of coordinates [goal_x, goal_y]
        self.map = None

        # Open and closed set
        self.open_set = []
        self.closed_set = []

    def reset(self, map):
        self.open_set.clear()
        self.closed_set.clear()
        self.map = map
        for x in range(self.map.width):
            for y in range(self.map.height):
                cell = self.map.get_cell(x,y)
                cell.g = 0
                cell.f = 0
                cell.h = 0
                cell.previous = None
            

    # Remove element from open set
    def pop_from_open_set(self, node_to_remove: Cell):
        for i,node in enumerate(self.open_set):
            if node == node_to_remove:
                self.open_set.pop(i)
                break

    # Compute heuristic score based on manhattan distance
    def h_score(self, current_node: Cell, goal_node: Cell):
        return abs(current_node.x - goal_node.x) + abs(current_node.y - goal_node.y)

    # A* core algorithm to find optimal path between start and goal
    # Returns a list of tuples (x,y) representing the coordinates of path waypoints
    def compute_path(self, start, goal):
        
        # Set initial node in open set and target node as goal
        self.open_set.append(self.map.get_cell(start[0], start[1]))
        goal_node = self.map.get_cell(goal[0], goal[1])

        path = []
        # Repeat as long as open set is not empty
        while self.open_set is not None:

            try:
                best_way = 0
                # Find node in open set with minimal cost
                for i, node in enumerate(self.open_set):
                    if node.f < self.open_set[best_way].f:
                        best_way = i

                current_node = self.open_set[best_way]

                # If goal reached, backtrack optimal path and return reversed path
                if current_node.is_equal(goal_node):
                    tmp = current_node
                    while tmp is not None:
                        path.append((tmp.x, tmp.y))
                        tmp = tmp.previous
                    return path[::-1]

                else:
                    # Remove current node from open set
                    self.pop_from_open_set(current_node)
                    self.closed_set.append(current_node)
                    # For each neighbour of the current node
                    for neighbour in current_node.neighbours:
                        # If neighbour is closed or obstacle, ignore it
                        if (neighbour in self.closed_set) or neighbour.empty == False:
                            continue
                        # If instead neighbour is empty space and not closed already...
                        else:
                            new_g = current_node.g + 1
                            already_in_open_set = False
                            # For each node alredy in open set
                            for node in self.open_set:
                                # Check if neighbour is already in open set
                                if neighbour.is_equal(node):
                                    # If so, update its cost if the new cost is smaller
                                    if new_g < node.g:
                                        node.g = new_g
                                        node.h = self.h_score(node, goal_node)
                                        node.f = node.g + node.h
                                        node.previous = current_node
                                    else:
                                        pass
                                    already_in_open_set = True
                            # If neighbour not in open set, add it
                            if not already_in_open_set:
                                neighbour.g = new_g
                                neighbour.h = self.h_score(neighbour, goal_node)
                                neighbour.f = neighbour.g + neighbour.h
                                neighbour.previous = current_node
                                self.open_set.append(neighbour)
            except Exception as e:
                return []