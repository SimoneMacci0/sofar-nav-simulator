import arcade

# Tile class to represent empty or occupied map space
class Cell(arcade.Sprite):

    def __init__(self, x, y, type, resource_path, size):
        super().__init__(resource_path)
        
        self.width = self.height = size
        self.x = x
        self.y = y
        # Cost variables
        self.f = 0
        self.g = 0
        self.h = 0
        # Neighbours
        self.neighbours = []
        # Prev node in optimal path
        self.previous = None
        # Is empty space or not
        self.empty = True if type == 0 else False

    def __repr__(self):
        return "Node({0}, {1})".format(self.x, self.y)

    def is_equal(self, other):
        return self.x == other.x and self.y == other.y
 

    