"""

A* grid planning

COEN 5830 HW

Full Name: ADD YOUR NAME HERE

"""

import math
import time

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.calc_obstacle_map(obstacle_x, obstacle_y)
        self.motion = self.get_motion_model()
        

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, start_x, start_y, goal_x, goal_y):

        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                               self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                              self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            current = None
           
            #Check if open_set is empty. If so, break out of the while loop
            if not open_set:
                print("Open set is empty, no path found!")
                return [], []  # If open_set is empty, break the loop            


            #Find the node in open_set with least cost to come (g) + cost to go (heuristic) and assign it to current
            current = min(open_set.values(), key=lambda node: node.cost + self.calc_heuristic(node, goal_node))

            #DO NOT ALTER THE NEXT 8 LINES
            if show_animation: 
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # If the current node is the goal node, path has been found
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the current node from the open set and add it to the closed set
            del open_set[self.calc_grid_index(current)]
            closed_set[self.calc_grid_index(current)] = current

            # Expand search based on the motion model
            for motion in self.motion:
                node_x = current.x + motion[0]
                node_y = current.y + motion[1]
                node_cost = current.cost + motion[2]
                node_index = self.calc_grid_index(self.Node(node_x, node_y, node_cost, current.parent_index))

                neighbor_node = self.Node(node_x, node_y, node_cost, self.calc_grid_index(current))

                if not self.verify_node(neighbor_node):
                    continue

                if node_index in closed_set:
                    continue

                # If the node is already in open_set, check if the cost is lower, if so, update it
                if node_index in open_set:
                    if open_set[node_index].cost > neighbor_node.cost:
                        open_set[node_index].cost = neighbor_node.cost
                        open_set[node_index].parent_index = self.calc_grid_index(current)
                else:
                    open_set[node_index] = neighbor_node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.x,node.y)

    def verify_node(self, node):
        #This function tries to verify if the node is within state space bounds and is collision-free
        #Verify if the node is within bounds and isn't colliding with an obstacle
        #Return False if node is invalid. Otherwise, return True
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        # Check if the node is outside the state space bounds
        if px < self.min_x or px >= self.max_x or py < self.min_y or py >= self.max_y:
            return False

        # Check if the node collides with an obstacle
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, obstacle_x, obstacle_y): 
        #This fuction is used to populate the obstacle map of the state space. The value is true if a specific cell in the map overlaps with an obstacle and false if not.
        #Find the minimum and maximum bounds for x and y
        #Use the bounds to obtain the width along x and y axes and store them in self.x_width and self.y_width respectively

        self.min_x = min(obstacle_x)
        self.min_y = min(obstacle_y)
        self.max_x = max(obstacle_x)
        self.max_y = max(obstacle_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        #DO NOT ALTER THE NEXT TWO LINES.
        self.obstacle_map = [[False for _ in range(self.y_width)]
                            for _ in range(self.x_width)]

        #For each cell in self.obstacle_map, use the calculations above to assign it as boolean True if the cell overlaps with an obstacle and boolean False if it doesn't 
        for ix in range(self.x_width):
            for iy in range(self.y_width):
                x = self.calc_grid_position(ix, self.min_x)
                y = self.calc_grid_position(iy, self.min_y)

                for ox, oy in zip(obstacle_x, obstacle_y):
                    d = math.sqrt((ox - x) ** 2 + (oy - y) ** 2)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break


    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    start_x = 5.0  # [m]
    start_y = 5.0  # [m]
    goal_x = 50.0  # [m]
    goal_y = 50.0  # [m]
    cell_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    #Feel free to change the obstacle positions and test the implementation on various scenarios
    obstacle_x, obstacle_y = [], []
    for i in range(0, 60):
        obstacle_x.append(i)
        obstacle_y.append(0.0)
    for i in range(0, 60):
        obstacle_x.append(60.0)
        obstacle_y.append(i)
    for i in range(0, 61):
        obstacle_x.append(i)
        obstacle_y.append(60.0)
    for i in range(0, 61):
        obstacle_x.append(0.0)
        obstacle_y.append(i)
    for i in range(0, 40):
        obstacle_x.append(20.0)
        obstacle_y.append(i)
    for i in range(0, 40):
        obstacle_x.append(40.0)
        obstacle_y.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(obstacle_x, obstacle_y, ".k")
        plt.plot(start_x, start_y, "og")
        plt.plot(goal_x, goal_y, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(obstacle_x, obstacle_y, cell_size, robot_radius)

    # Measure time for Dijkstra's algorithm
    start_time = time.time()
    rx, ry = a_star.planning(start_x, start_y, goal_x, goal_y)
    end_time = time.time()

    print(f"A*'s Time: {end_time - start_time:.4f} seconds")


    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.1)
        plt.show()



if __name__ == '__main__':
    main()

