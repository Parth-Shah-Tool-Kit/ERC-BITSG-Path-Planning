import time
import math
import matplotlib
from matplotlib import pyplot
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from random import randint, randrange

width = 100
height = 100

class Obstacle:
    def __init__(self):
        self.x, self.y = randrange(5, 95), randrange(5, 95)
        self.w, self.h = randrange(5, 20), randrange(5, 20)
        
        if self.x + self.w > width:
            self.w = width - self.x
        
        if self.y + self.h > height:
            self.h = height - self.y

        self.coords = [(self.x, self.y), (self.x + self.w, self.y), (self.x + self.w, self.y + self.h), (self.x, self.y + self.h)]
        self.poly = Polygon(self.coords)


class RRT:
    def __init__(self, n_obs,steps, max_size):
        self.figure = pyplot.figure(figsize=(15,12))
        
        self.plot = self.figure.add_subplot(111)
         
        
        self.max_size = max_size
        self.start = Point(0,100)
        self.end = Point(100,0)
        self.obstacles = []
        
        self.paths = [[self.start]]
        self.cur_path = [self.start]
        self.nodes = [self.start]

        self.generate_boundary()
        self.plot.scatter([self.start.x, self.end.x], [self.start.y, self.end.y], color="red", s=100)
        self.generate_obstacles(n_obs)
        self.plan_path(steps)

    def generate_obstacles(self, num_of_obs):
        for _ in range(num_of_obs):
            obs = Obstacle()
            rect = matplotlib.patches.Rectangle((obs.x, obs.y),
                                                obs.w, obs.h,
                                                color='black')
            self.plot.add_patch(rect)
            self.obstacles.append(obs.poly)

    def generate_boundary(self):
        rect1 = matplotlib.patches.Rectangle((-5, -5), 3, 110, color='black')
        rect2 = matplotlib.patches.Rectangle((-5, 102), 110, 5, color='black')
        rect3 = matplotlib.patches.Rectangle((102, -5), 5, 110, color='black')
        rect4 = matplotlib.patches.Rectangle((-5, -5), 110, 3, color='black')
        self.plot.add_patch(rect1)
        self.plot.add_patch(rect2)
        self.plot.add_patch(rect3)
        self.plot.add_patch(rect4)

    def plan_path(self, no_of_steps):
        for _ in range(no_of_steps):
            found = False
            while not found:
                random_point = Point(randrange(0, 100), randrange(0, 100))
                nearest_neighbor, node = self.find_nearest_neighbor(random_point)
                if self.check_intersections(nearest_neighbor, node):
                    self.nodes.append(node)
                    self.plot.scatter([node.x], [node.y], color="blue", s=50)
                    self.plot.plot([nearest_neighbor.x, node.x], [nearest_neighbor.y, node.y], color="orange")
                    pyplot.pause(0.001)
                    found = True
                    for path in self.paths:
                        if path[-1] == nearest_neighbor:
                            self.paths.append(path+[node])
                            self.cur_path = list(path+[node])
            
            if self.check_intersections(node, self.end):
                self.plot.plot([node.x, self.end.x], [node.y, self.end.y], color="orange")
                pyplot.pause(0.001)
                self.cur_path.append(self.end)
                break
            

            if node.distance(self.end) < 5:
                self.cur_path.append(self.end)
                break

        for i in range(len(self.cur_path)-1):
            cur_point = self.cur_path[i]
            nxt_point = self.cur_path[i+1]
            self.plot.scatter([cur_point.x], [cur_point.y], color="#90EE90", s=50)
            self.plot.plot([cur_point.x, nxt_point.x], [cur_point.y, nxt_point.y], color="#90EE90")
            # pyplot.pause(0.001)

        pyplot.pause(0.001)
        return

    def find_nearest_neighbor(self, point):
        distances = [point.distance(path_point) for path_point in self.nodes]
        d = min(distances)
        p1 = self.nodes[distances.index(d)]
        if d <= self.max_size:
            return p1, point
        else:
            x1, x2, y1, y2 = p1.x, point.x, p1.y, point.y
            if x2 != x1:
                m = (y2 - y1) / (x2 - x1)
                cos_theta = 1/(math.sqrt(1 + m**2))
                sin_theta = m/(math.sqrt(1 + m**2))
                new_x = x1 + (self.max_size*cos_theta)
                new_y = y1 + (self.max_size*sin_theta)
                new_node = Point(new_x, new_y)
            else:
                new_y = y1 + self.max_step_size
                new_x = x1
                new_node = Point(new_x, new_y)

            return p1, new_node

    def check_intersections(self, p1, p2):
        valid = True
        line = LineString([(p1.x, p1.y), (p2.x, p2.y)])
        for obs in self.obstacles:
            if line.intersection(obs):
                valid = not valid
                break

        return valid

        

output = RRT(25,1000,5)
pyplot.show()
