"""
COEN 5830

Homework #2 (Assigned: 9/22, Due: 10/4 11:59pm on Canvas)
Motion Planning

Full Name: Jusung Kim

Collaborators : Jacqulin Justin
"""

# Q1. [5 points] A template has been provided to implement and visualize Dijkstra’s algorithm via dijkstra.py. A brief psuedocode has been commented in the python file to help in the implementation. Finish the implementation such that the following output is obtained


# import time
# import matplotlib.pyplot as plt
# import math

# show_animation = True


# class Dijkstra:

#     def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):

#         self.min_x = None
#         self.min_y = None
#         self.max_x = None
#         self.max_y = None
#         self.x_width = None
#         self.y_width = None
#         self.obstacle_map = None
        
#         self.resolution = resolution
#         self.robot_radius = robot_radius
#         self.calc_obstacle_map(obstacle_x, obstacle_y)
#         self.motion = self.get_motion_model()

#     class Node:
#         def __init__(self, x, y, cost, parent_index):
#             self.x = x  # index of grid
#             self.y = y  # index of grid
#             self.cost = cost
#             self.parent_index = parent_index  # index of previous Node

#     def planning(self, start_x, start_y, goal_x, goal_y):

#         start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
#                                self.calc_xy_index(start_y, self.min_y), 0.0, -1)
#         goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
#                               self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

#         open_set, closed_set = dict(), dict()
#         open_set[self.calc_grid_index(start_node)] = start_node


#         while True:
#             #Find the Node in open_set with least cost and assign it to current
#             current = min(open_set.values(), key=lambda n: n.cost)
            
#             if current.x == goal_node.x and current.y == goal_node.y:
#                 goal_node.parent_index = current.parent_index
#                 goal_node.cost = current.cost
#                 break
            
#             del open_set[self.calc_grid_index(current)]
#             closed_set[self.calc_grid_index(current)] = current
            
#             for move_x, move_y, move_cost in self.motion:
#                 node_x = current.x + move_x
#                 node_y = current.y + move_y
#                 node = self.Node(node_x, node_y, current.cost + move_cost, self.calc_grid_index(current))
                
#                 if not self.verify_node(node):
#                     continue
#                 node_index = self.calc_grid_index(node)
                
#                 if node_index in closed_set:
#                     continue
                
#                 if node_index not in open_set:
#                     open_set[node_index] = node
#                 else:
#                     if open_set[node_index].cost > node.cost:
#                         open_set[node_index] = node


#             #DO NOT ALTER THE NEXT 8 LINES. 
#             if show_animation:  # pragma: no cover
#                 plt.plot(self.calc_grid_position(current.x, self.min_x),
#                          self.calc_grid_position(current.y, self.min_y), "xc")
#                 plt.gcf().canvas.mpl_connect(
#                     'key_release_event',
#                     lambda event: [exit(0) if event.key == 'escape' else None])
#                 if len(closed_set.keys()) % 10 == 0:
#                     plt.pause(0.001)

#             # Check if the current Node is the goal node. If so, assign appropriate values to goal_node.parent_index and goal_node.cost and break from the while loop
#             # If the current node is not the goal, remove the item from the open set and add it to the closed set
#             # Use the motion model to expand the search to other neighbors in the grid.
#             # Check if the neighbouring cell is a part of closed set. If so, move on.
#             # If the neighboring cell is not within bounds of the state space, move on.
#             # If the neighboring cell is neither in open_set or closed_set, add it to the open set.
#             # If the neighboring cell is a part of the open cell, will expanding from the current node reduce the total cost to reach the neighbor? If so, replace it with the current node. (Essentially changing its parent and cost). 

#         rx, ry = self.calc_final_path(goal_node, closed_set)

#         return rx, ry

#     def calc_final_path(self, goal_node, closed_set):
#         rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
#             self.calc_grid_position(goal_node.y, self.min_y)]
#         parent_index = goal_node.parent_index
#         while parent_index != -1:
#             n = closed_set[parent_index]
#             rx.append(self.calc_grid_position(n.x, self.min_x))
#             ry.append(self.calc_grid_position(n.y, self.min_y))
#             parent_index = n.parent_index

#         return rx, ry

#     def calc_grid_position(self, index, min_position):
#         pos = index * self.resolution + min_position
#         return pos

#     def calc_xy_index(self, position, min_position):
#         return round((position - min_position) / self.resolution)

#     def calc_grid_index(self, node):
#         return (node.x,node.y)
        

#     def verify_node(self, node):
#         #This function tries to verify if the node is within state space bounds and is collision-free
#         #Verify if the node is within bounds and isn't colliding with an obstacle
#         #Return False if node is invalid. Otherwise, return True
#         px = self.calc_grid_position(node.x, self.min_x)
#         py = self.calc_grid_position(node.y, self.min_y)

#         # Check if the node is outside the state space bounds
#         if px < self.min_x or px >= self.max_x or py < self.min_y or py >= self.max_y:
#             return False

#         # Check if the node collides with an obstacle
#         if self.obstacle_map[node.x][node.y]:
#             return False

#         return True


#     def calc_obstacle_map(self, obstacle_x, obstacle_y): 
#         #This fuction is used to populate the obstacle map of the state space. The value is true if a specific cell in the map overlaps with an obstacle and false if not.
#         #Find the minimum and maximum bounds for x and y
#         #Use the bounds to obtain the width along x and y axes and store them in self.x_width and self.y_width respectively

#         self.min_x = min(obstacle_x)
#         self.min_y = min(obstacle_y)
#         self.max_x = max(obstacle_x)
#         self.max_y = max(obstacle_y)

#         self.x_width = round((self.max_x - self.min_x) / self.resolution)
#         self.y_width = round((self.max_y - self.min_y) / self.resolution)

#         #DO NOT ALTER THE NEXT TWO LINES.
#         self.obstacle_map = [[False for _ in range(self.y_width)]
#                             for _ in range(self.x_width)]

#         #For each cell in self.obstacle_map, use the calculations above to assign it as boolean True if the cell overlaps with an obstacle and boolean False if it doesn't 
#         for ix in range(self.x_width):
#             for iy in range(self.y_width):
#                 x = self.calc_grid_position(ix, self.min_x)
#                 y = self.calc_grid_position(iy, self.min_y)

#                 for ox, oy in zip(obstacle_x, obstacle_y):
#                     d = math.sqrt((ox - x) ** 2 + (oy - y) ** 2)
#                     if d <= self.robot_radius:
#                         self.obstacle_map[ix][iy] = True
#                         break



#     @staticmethod
#     def get_motion_model():
#         # dx, dy, cost
#         motion = [[1, 0, 1],
#                   [0, 1, 1],
#                   [-1, 0, 1],
#                   [0, -1, 1],
#                   [-1, -1, math.sqrt(2)],
#                   [-1, 1, math.sqrt(2)],
#                   [1, -1, math.sqrt(2)],
#                   [1, 1, math.sqrt(2)]]

#         return motion


# def main():
#     print(__file__ + " start!!")

#     # start and goal position
#     start_x = 5.0  # [m]
#     start_y = 5.0  # [m]
#     goal_x = 50.0  # [m]
#     goal_y = 50.0  # [m]
#     cell_size = 2.0  # [m]
#     robot_radius = 1.0  # [m]

#     #Feel free to change the obstacle positions and test the implementation on various scenarios
#     obstacle_x, obstacle_y = [], []
#     for i in range(0, 60):
#         obstacle_x.append(i)
#         obstacle_y.append(0.0)
#     for i in range(0, 60):
#         obstacle_x.append(60.0)
#         obstacle_y.append(i)
#     for i in range(0, 61):
#         obstacle_x.append(i)
#         obstacle_y.append(60.0)
#     for i in range(0, 61):
#         obstacle_x.append(0.0)
#         obstacle_y.append(i)
#     for i in range(0, 40):
#         obstacle_x.append(20.0)
#         obstacle_y.append(i)
#     for i in range(0, 40):
#         obstacle_x.append(40.0)
#         obstacle_y.append(60.0 - i)





#     if show_animation: 
#         plt.plot(obstacle_x, obstacle_y, ".k")
#         plt.plot(start_x, start_y, "og")
#         plt.plot(goal_x, goal_y, "xb")
#         plt.grid(True)
#         plt.axis("equal")

#     dijkstra = Dijkstra(obstacle_x, obstacle_y, cell_size, robot_radius)


#     # Measure time for Dijkstra's algorithm
#     start_time = time.time()
#     rx, ry = dijkstra.planning(start_x, start_y, goal_x, goal_y)
#     end_time = time.time()

#     print(f"Dijkstra's Time: {end_time - start_time:.4f} seconds")


#     if show_animation:
#         plt.plot(rx, ry, "-r")
#         plt.pause(0.01)
#         plt.show()


# if __name__ == '__main__':
#     main()




# Q2. [5 points] A template has been provided to implement and visualize A* algorithm via a star.py. A brief psuedocode has been commented in the python file to help in the implementation.Finish the implementation such that the following output is obtained:


# import math
# import time
# import matplotlib.pyplot as plt
# show_animation = True


# class AStarPlanner:

#     def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):

#         self.resolution = resolution
#         self.robot_radius = robot_radius
#         self.min_x, self.min_y = 0, 0
#         self.max_x, self.max_y = 0, 0
#         self.obstacle_map = None
#         self.x_width, self.y_width = 0, 0
#         self.calc_obstacle_map(obstacle_x, obstacle_y)
#         self.motion = self.get_motion_model()
        

#     class Node:
#         def __init__(self, x, y, cost, parent_index):
#             self.x = x  # index of grid
#             self.y = y  # index of grid
#             self.cost = cost
#             self.parent_index = parent_index

#     def planning(self, start_x, start_y, goal_x, goal_y):

#         start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
#                                self.calc_xy_index(start_y, self.min_y), 0.0, -1)
#         goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
#                               self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

#         open_set, closed_set = dict(), dict()
#         open_set[self.calc_grid_index(start_node)] = start_node

#         while True:
#             current = None
           
#             #Check if open_set is empty. If so, break out of the while loop
#             if not open_set:
#                 print("Open set is empty, no path found!")
#                 return [], []  # If open_set is empty, break the loop            


#             #Find the node in open_set with least cost to come (g) + cost to go (heuristic) and assign it to current
#             current = min(open_set.values(), key=lambda node: node.cost + self.calc_heuristic(node, goal_node))

#             #DO NOT ALTER THE NEXT 8 LINES
#             if show_animation: 
#                 plt.plot(self.calc_grid_position(current.x, self.min_x),
#                          self.calc_grid_position(current.y, self.min_y), "xc")
#                 plt.gcf().canvas.mpl_connect('key_release_event',
#                                              lambda event: [exit(
#                                                  0) if event.key == 'escape' else None])
#                 if len(closed_set.keys()) % 10 == 0:
#                     plt.pause(0.001)

#             # If the current node is the goal node, path has been found
#             if current.x == goal_node.x and current.y == goal_node.y:
#                 goal_node.parent_index = current.parent_index
#                 goal_node.cost = current.cost
#                 break

#             # Remove the current node from the open set and add it to the closed set
#             del open_set[self.calc_grid_index(current)]
#             closed_set[self.calc_grid_index(current)] = current

#             # Expand search based on the motion model
#             for motion in self.motion:
#                 node_x = current.x + motion[0]
#                 node_y = current.y + motion[1]
#                 node_cost = current.cost + motion[2]
#                 node_index = self.calc_grid_index(self.Node(node_x, node_y, node_cost, current.parent_index))

#                 neighbor_node = self.Node(node_x, node_y, node_cost, self.calc_grid_index(current))

#                 if not self.verify_node(neighbor_node):
#                     continue

#                 if node_index in closed_set:
#                     continue

#                 # If the node is already in open_set, check if the cost is lower, if so, update it
#                 if node_index in open_set:
#                     if open_set[node_index].cost > neighbor_node.cost:
#                         open_set[node_index].cost = neighbor_node.cost
#                         open_set[node_index].parent_index = self.calc_grid_index(current)
#                 else:
#                     open_set[node_index] = neighbor_node

#         rx, ry = self.calc_final_path(goal_node, closed_set)
#         return rx, ry

#     def calc_final_path(self, goal_node, closed_set):
#         # generate final course
#         rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
#             self.calc_grid_position(goal_node.y, self.min_y)]
#         parent_index = goal_node.parent_index
#         while parent_index != -1:
#             n = closed_set[parent_index]
#             rx.append(self.calc_grid_position(n.x, self.min_x))
#             ry.append(self.calc_grid_position(n.y, self.min_y))
#             parent_index = n.parent_index

#         return rx, ry

#     @staticmethod
#     def calc_heuristic(n1, n2):
#         w = 1.0  # weight of heuristic
#         d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
#         return d

#     def calc_grid_position(self, index, min_position):
#         pos = index * self.resolution + min_position
#         return pos

#     def calc_xy_index(self, position, min_pos):
#         return round((position - min_pos) / self.resolution)

#     def calc_grid_index(self, node):
#         return (node.x,node.y)

#     def verify_node(self, node):
#         #This function tries to verify if the node is within state space bounds and is collision-free
#         #Verify if the node is within bounds and isn't colliding with an obstacle
#         #Return False if node is invalid. Otherwise, return True
#         px = self.calc_grid_position(node.x, self.min_x)
#         py = self.calc_grid_position(node.y, self.min_y)

#         # Check if the node is outside the state space bounds
#         if px < self.min_x or px >= self.max_x or py < self.min_y or py >= self.max_y:
#             return False

#         # Check if the node collides with an obstacle
#         if self.obstacle_map[node.x][node.y]:
#             return False

#         return True

#     def calc_obstacle_map(self, obstacle_x, obstacle_y): 
#         #This fuction is used to populate the obstacle map of the state space. The value is true if a specific cell in the map overlaps with an obstacle and false if not.
#         #Find the minimum and maximum bounds for x and y
#         #Use the bounds to obtain the width along x and y axes and store them in self.x_width and self.y_width respectively

#         self.min_x = min(obstacle_x)
#         self.min_y = min(obstacle_y)
#         self.max_x = max(obstacle_x)
#         self.max_y = max(obstacle_y)

#         self.x_width = round((self.max_x - self.min_x) / self.resolution)
#         self.y_width = round((self.max_y - self.min_y) / self.resolution)

#         #DO NOT ALTER THE NEXT TWO LINES.
#         self.obstacle_map = [[False for _ in range(self.y_width)]
#                             for _ in range(self.x_width)]

#         #For each cell in self.obstacle_map, use the calculations above to assign it as boolean True if the cell overlaps with an obstacle and boolean False if it doesn't 
#         for ix in range(self.x_width):
#             for iy in range(self.y_width):
#                 x = self.calc_grid_position(ix, self.min_x)
#                 y = self.calc_grid_position(iy, self.min_y)

#                 for ox, oy in zip(obstacle_x, obstacle_y):
#                     d = math.sqrt((ox - x) ** 2 + (oy - y) ** 2)
#                     if d <= self.robot_radius:
#                         self.obstacle_map[ix][iy] = True
#                         break


#     @staticmethod
#     def get_motion_model():
#         # dx, dy, cost
#         motion = [[1, 0, 1],
#                   [0, 1, 1],
#                   [-1, 0, 1],
#                   [0, -1, 1],
#                   [-1, -1, math.sqrt(2)],
#                   [-1, 1, math.sqrt(2)],
#                   [1, -1, math.sqrt(2)],
#                   [1, 1, math.sqrt(2)]]

#         return motion


# def main():
#     print(__file__ + " start!!")

#     # start and goal position
#     start_x = 5.0  # [m]
#     start_y = 5.0  # [m]
#     goal_x = 50.0  # [m]
#     goal_y = 50.0  # [m]
#     cell_size = 2.0  # [m]
#     robot_radius = 1.0  # [m]

#     #Feel free to change the obstacle positions and test the implementation on various scenarios
#     obstacle_x, obstacle_y = [], []
#     for i in range(0, 60):
#         obstacle_x.append(i)
#         obstacle_y.append(0.0)
#     for i in range(0, 60):
#         obstacle_x.append(60.0)
#         obstacle_y.append(i)
#     for i in range(0, 61):
#         obstacle_x.append(i)
#         obstacle_y.append(60.0)
#     for i in range(0, 61):
#         obstacle_x.append(0.0)
#         obstacle_y.append(i)
#     for i in range(0, 40):
#         obstacle_x.append(20.0)
#         obstacle_y.append(i)
#     for i in range(0, 40):
#         obstacle_x.append(40.0)
#         obstacle_y.append(60.0 - i)

#     if show_animation:  # pragma: no cover
#         plt.plot(obstacle_x, obstacle_y, ".k")
#         plt.plot(start_x, start_y, "og")
#         plt.plot(goal_x, goal_y, "xb")
#         plt.grid(True)
#         plt.axis("equal")

#     a_star = AStarPlanner(obstacle_x, obstacle_y, cell_size, robot_radius)

#     # Measure time for Dijkstra's algorithm
#     start_time = time.time()
#     rx, ry = a_star.planning(start_x, start_y, goal_x, goal_y)
#     end_time = time.time()

#     print(f"A*'s Time: {end_time - start_time:.4f} seconds")


#     if show_animation:  # pragma: no cover
#         plt.plot(rx, ry, "-r")
#         plt.pause(0.1)
#         plt.show()



# if __name__ == '__main__':
#     main()








# Q3.[10 points] Change the environments for the A* and Dijkstra’s implementation such that the time taken for both the algorithms is the same. Use python’s time library to show that the time taken by both is approximately the same (±0.2 seconds) . (Note: The heuristic function must not be altered.



# import time
# import matplotlib.pyplot as plt
# import math

# show_animation = True


# class Dijkstra:

#     def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):

#         self.min_x = None
#         self.min_y = None
#         self.max_x = None
#         self.max_y = None
#         self.x_width = None
#         self.y_width = None
#         self.obstacle_map = None
        
#         self.resolution = resolution
#         self.robot_radius = robot_radius
#         self.calc_obstacle_map(obstacle_x, obstacle_y)
#         self.motion = self.get_motion_model()

#     class Node:
#         def __init__(self, x, y, cost, parent_index):
#             self.x = x  # index of grid
#             self.y = y  # index of grid
#             self.cost = cost
#             self.parent_index = parent_index  # index of previous Node

#     def planning(self, start_x, start_y, goal_x, goal_y):

#         start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
#                                self.calc_xy_index(start_y, self.min_y), 0.0, -1)
#         goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
#                               self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

#         open_set, closed_set = dict(), dict()
#         open_set[self.calc_grid_index(start_node)] = start_node


#         while True:
#             #Find the Node in open_set with least cost and assign it to current
#             current = min(open_set.values(), key=lambda n: n.cost)
            
#             if current.x == goal_node.x and current.y == goal_node.y:
#                 goal_node.parent_index = current.parent_index
#                 goal_node.cost = current.cost
#                 break
            
#             del open_set[self.calc_grid_index(current)]
#             closed_set[self.calc_grid_index(current)] = current
            
#             for move_x, move_y, move_cost in self.motion:
#                 node_x = current.x + move_x
#                 node_y = current.y + move_y
#                 node = self.Node(node_x, node_y, current.cost + move_cost, self.calc_grid_index(current))
                
#                 if not self.verify_node(node):
#                     continue
#                 node_index = self.calc_grid_index(node)
                
#                 if node_index in closed_set:
#                     continue
                
#                 if node_index not in open_set:
#                     open_set[node_index] = node
#                 else:
#                     if open_set[node_index].cost > node.cost:
#                         open_set[node_index] = node


#             #DO NOT ALTER THE NEXT 8 LINES. 
#             if show_animation:  # pragma: no cover
#                 plt.plot(self.calc_grid_position(current.x, self.min_x),
#                          self.calc_grid_position(current.y, self.min_y), "xc")
#                 plt.gcf().canvas.mpl_connect(
#                     'key_release_event',
#                     lambda event: [exit(0) if event.key == 'escape' else None])
#                 if len(closed_set.keys()) % 10 == 0:
#                     plt.pause(0.001)

#             # Check if the current Node is the goal node. If so, assign appropriate values to goal_node.parent_index and goal_node.cost and break from the while loop
#             # If the current node is not the goal, remove the item from the open set and add it to the closed set
#             # Use the motion model to expand the search to other neighbors in the grid.
#             # Check if the neighbouring cell is a part of closed set. If so, move on.
#             # If the neighboring cell is not within bounds of the state space, move on.
#             # If the neighboring cell is neither in open_set or closed_set, add it to the open set.
#             # If the neighboring cell is a part of the open cell, will expanding from the current node reduce the total cost to reach the neighbor? If so, replace it with the current node. (Essentially changing its parent and cost). 

#         rx, ry = self.calc_final_path(goal_node, closed_set)

#         return rx, ry

#     def calc_final_path(self, goal_node, closed_set):
#         rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
#             self.calc_grid_position(goal_node.y, self.min_y)]
#         parent_index = goal_node.parent_index
#         while parent_index != -1:
#             n = closed_set[parent_index]
#             rx.append(self.calc_grid_position(n.x, self.min_x))
#             ry.append(self.calc_grid_position(n.y, self.min_y))
#             parent_index = n.parent_index

#         return rx, ry

#     def calc_grid_position(self, index, min_position):
#         pos = index * self.resolution + min_position
#         return pos

#     def calc_xy_index(self, position, min_position):
#         return round((position - min_position) / self.resolution)

#     def calc_grid_index(self, node):
#         return (node.x,node.y)
        

#     def verify_node(self, node):
#         #This function tries to verify if the node is within state space bounds and is collision-free
#         #Verify if the node is within bounds and isn't colliding with an obstacle
#         #Return False if node is invalid. Otherwise, return True
#         px = self.calc_grid_position(node.x, self.min_x)
#         py = self.calc_grid_position(node.y, self.min_y)

#         # Check if the node is outside the state space bounds
#         if px < self.min_x or px >= self.max_x or py < self.min_y or py >= self.max_y:
#             return False

#         # Check if the node collides with an obstacle
#         if self.obstacle_map[node.x][node.y]:
#             return False

#         return True


#     def calc_obstacle_map(self, obstacle_x, obstacle_y): 
#         #This fuction is used to populate the obstacle map of the state space. The value is true if a specific cell in the map overlaps with an obstacle and false if not.
#         #Find the minimum and maximum bounds for x and y
#         #Use the bounds to obtain the width along x and y axes and store them in self.x_width and self.y_width respectively

#         self.min_x = min(obstacle_x)
#         self.min_y = min(obstacle_y)
#         self.max_x = max(obstacle_x)
#         self.max_y = max(obstacle_y)

#         self.x_width = round((self.max_x - self.min_x) / self.resolution)
#         self.y_width = round((self.max_y - self.min_y) / self.resolution)

#         #DO NOT ALTER THE NEXT TWO LINES.
#         self.obstacle_map = [[False for _ in range(self.y_width)]
#                             for _ in range(self.x_width)]

#         #For each cell in self.obstacle_map, use the calculations above to assign it as boolean True if the cell overlaps with an obstacle and boolean False if it doesn't 
#         for ix in range(self.x_width):
#             for iy in range(self.y_width):
#                 x = self.calc_grid_position(ix, self.min_x)
#                 y = self.calc_grid_position(iy, self.min_y)

#                 for ox, oy in zip(obstacle_x, obstacle_y):
#                     d = math.sqrt((ox - x) ** 2 + (oy - y) ** 2)
#                     if d <= self.robot_radius:
#                         self.obstacle_map[ix][iy] = True
#                         break



#     @staticmethod
#     def get_motion_model():
#         # dx, dy, cost
#         motion = [[1, 0, 1],
#                   [0, 1, 1],
#                   [-1, 0, 1],
#                   [0, -1, 1],
#                   [-1, -1, math.sqrt(2)],
#                   [-1, 1, math.sqrt(2)],
#                   [1, -1, math.sqrt(2)],
#                   [1, 1, math.sqrt(2)]]

#         return motion
    

# class AStarPlanner:

#     def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):

#         self.resolution = resolution
#         self.robot_radius = robot_radius
#         self.min_x, self.min_y = 0, 0
#         self.max_x, self.max_y = 0, 0
#         self.obstacle_map = None
#         self.x_width, self.y_width = 0, 0
#         self.calc_obstacle_map(obstacle_x, obstacle_y)
#         self.motion = self.get_motion_model()
        

#     class Node:
#         def __init__(self, x, y, cost, parent_index):
#             self.x = x  # index of grid
#             self.y = y  # index of grid
#             self.cost = cost
#             self.parent_index = parent_index

#     def planning(self, start_x, start_y, goal_x, goal_y):

#         start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
#                                self.calc_xy_index(start_y, self.min_y), 0.0, -1)
#         goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
#                               self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

#         open_set, closed_set = dict(), dict()
#         open_set[self.calc_grid_index(start_node)] = start_node

#         while True:
#             current = None
           
#             #Check if open_set is empty. If so, break out of the while loop
#             if not open_set:
#                 print("Open set is empty, no path found!")
#                 return [], []  # If open_set is empty, break the loop            


#             #Find the node in open_set with least cost to come (g) + cost to go (heuristic) and assign it to current
#             current = min(open_set.values(), key=lambda node: node.cost + self.calc_heuristic(node, goal_node))

#             #DO NOT ALTER THE NEXT 8 LINES
#             if show_animation: 
#                 plt.plot(self.calc_grid_position(current.x, self.min_x),
#                          self.calc_grid_position(current.y, self.min_y), "xc")
#                 plt.gcf().canvas.mpl_connect('key_release_event',
#                                              lambda event: [exit(
#                                                  0) if event.key == 'escape' else None])
#                 if len(closed_set.keys()) % 10 == 0:
#                     plt.pause(0.001)

#             # If the current node is the goal node, path has been found
#             if current.x == goal_node.x and current.y == goal_node.y:
#                 goal_node.parent_index = current.parent_index
#                 goal_node.cost = current.cost
#                 break

#             # Remove the current node from the open set and add it to the closed set
#             del open_set[self.calc_grid_index(current)]
#             closed_set[self.calc_grid_index(current)] = current

#             # Expand search based on the motion model
#             for motion in self.motion:
#                 node_x = current.x + motion[0]
#                 node_y = current.y + motion[1]
#                 node_cost = current.cost + motion[2]
#                 node_index = self.calc_grid_index(self.Node(node_x, node_y, node_cost, current.parent_index))

#                 neighbor_node = self.Node(node_x, node_y, node_cost, self.calc_grid_index(current))

#                 if not self.verify_node(neighbor_node):
#                     continue

#                 if node_index in closed_set:
#                     continue

#                 # If the node is already in open_set, check if the cost is lower, if so, update it
#                 if node_index in open_set:
#                     if open_set[node_index].cost > neighbor_node.cost:
#                         open_set[node_index].cost = neighbor_node.cost
#                         open_set[node_index].parent_index = self.calc_grid_index(current)
#                 else:
#                     open_set[node_index] = neighbor_node

#         rx, ry = self.calc_final_path(goal_node, closed_set)
#         return rx, ry

#     def calc_final_path(self, goal_node, closed_set):
#         # generate final course
#         rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
#             self.calc_grid_position(goal_node.y, self.min_y)]
#         parent_index = goal_node.parent_index
#         while parent_index != -1:
#             n = closed_set[parent_index]
#             rx.append(self.calc_grid_position(n.x, self.min_x))
#             ry.append(self.calc_grid_position(n.y, self.min_y))
#             parent_index = n.parent_index

#         return rx, ry

#     @staticmethod
#     def calc_heuristic(n1, n2):
#         w = 1.0  # weight of heuristic
#         d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
#         return d

#     def calc_grid_position(self, index, min_position):
#         pos = index * self.resolution + min_position
#         return pos

#     def calc_xy_index(self, position, min_pos):
#         return round((position - min_pos) / self.resolution)

#     def calc_grid_index(self, node):
#         return (node.x,node.y)

#     def verify_node(self, node):
#         #This function tries to verify if the node is within state space bounds and is collision-free
#         #Verify if the node is within bounds and isn't colliding with an obstacle
#         #Return False if node is invalid. Otherwise, return True
#         px = self.calc_grid_position(node.x, self.min_x)
#         py = self.calc_grid_position(node.y, self.min_y)

#         # Check if the node is outside the state space bounds
#         if px < self.min_x or px >= self.max_x or py < self.min_y or py >= self.max_y:
#             return False

#         # Check if the node collides with an obstacle
#         if self.obstacle_map[node.x][node.y]:
#             return False

#         return True

#     def calc_obstacle_map(self, obstacle_x, obstacle_y): 
#         #This fuction is used to populate the obstacle map of the state space. The value is true if a specific cell in the map overlaps with an obstacle and false if not.
#         #Find the minimum and maximum bounds for x and y
#         #Use the bounds to obtain the width along x and y axes and store them in self.x_width and self.y_width respectively

#         self.min_x = min(obstacle_x)
#         self.min_y = min(obstacle_y)
#         self.max_x = max(obstacle_x)
#         self.max_y = max(obstacle_y)

#         self.x_width = round((self.max_x - self.min_x) / self.resolution)
#         self.y_width = round((self.max_y - self.min_y) / self.resolution)

#         #DO NOT ALTER THE NEXT TWO LINES.
#         self.obstacle_map = [[False for _ in range(self.y_width)]
#                             for _ in range(self.x_width)]

#         #For each cell in self.obstacle_map, use the calculations above to assign it as boolean True if the cell overlaps with an obstacle and boolean False if it doesn't 
#         for ix in range(self.x_width):
#             for iy in range(self.y_width):
#                 x = self.calc_grid_position(ix, self.min_x)
#                 y = self.calc_grid_position(iy, self.min_y)

#                 for ox, oy in zip(obstacle_x, obstacle_y):
#                     d = math.sqrt((ox - x) ** 2 + (oy - y) ** 2)
#                     if d <= self.robot_radius:
#                         self.obstacle_map[ix][iy] = True
#                         break


#     @staticmethod
#     def get_motion_model():
#         # dx, dy, cost
#         motion = [[1, 0, 1],
#                   [0, 1, 1],
#                   [-1, 0, 1],
#                   [0, -1, 1],
#                   [-1, -1, math.sqrt(2)],
#                   [-1, 1, math.sqrt(2)],
#                   [1, -1, math.sqrt(2)],
#                   [1, 1, math.sqrt(2)]]

#         return motion



# def main():
#     print(__file__ + " start!!")

#     # start and goal position
#     start_x = 5.0  # [m]
#     start_y = 5.0  # [m]
#     goal_x = 50.0  # [m]
#     goal_y = 50.0  # [m]
#     cell_size = 2.0  # [m]
#     robot_radius = 1.0  # [m]

#     # Feel free to change the obstacle positions and test the implementation on various scenarios
#     obstacle_x, obstacle_y = [], []
    
#     # box
#     for i in range(0, 60):
#         obstacle_x.append(i)
#         obstacle_y.append(0.0)
#     for i in range(0, 60):
#         obstacle_x.append(60.0)
#         obstacle_y.append(i)
#     for i in range(0, 61):
#         obstacle_x.append(i)
#         obstacle_y.append(60.0)
#     for i in range(0, 61):
#         obstacle_x.append(0.0)
#         obstacle_y.append(i)

#     # inner lines
#     for i in range(0, 50):
#         obstacle_x.append(i)
#         obstacle_y.append(6.0)
    
#     for i in range(6, 48):
#         obstacle_x.append(50.0)
#         obstacle_y.append(i)
#     for i in range(0, 48):
#         obstacle_x.append(55.0)
#         obstacle_y.append(i)

#     if show_animation:  # pragma: no cover
#         plt.plot(obstacle_x, obstacle_y, ".k")
#         plt.plot(start_x, start_y, "og")
#         plt.plot(goal_x, goal_y, "xb")
#         plt.grid(True)
#         plt.axis("equal")

#     # A* algorithm
#     a_star = AStarPlanner(obstacle_x, obstacle_y, cell_size, robot_radius)

#     start_time = time.time()
#     rx_astar, ry_astar = a_star.planning(start_x, start_y, goal_x, goal_y)
#     end_time = time.time()

#     # Dijkstra's algorithm
#     dijkstra = Dijkstra(obstacle_x, obstacle_y, cell_size, robot_radius)

#     start_time = time.time()
#     rx_dijkstra, ry_dijkstra = dijkstra.planning(start_x, start_y, goal_x, goal_y)
#     end_time = time.time()

#     print(f"Dijkstra's Time: {end_time - start_time:.4f} seconds")

#     if show_animation:  # pragma: no cover
#         plt.plot(rx_dijkstra, ry_dijkstra, "-r", label="Dijkstra")



#     print(f"A*'s Time: {end_time - start_time:.4f} seconds")

#     if show_animation:  # pragma: no cover
#         plt.plot(rx_astar, ry_astar, "-b", label="A*")
#         plt.legend()
#         plt.pause(0.1)
#         plt.show()

# if __name__ == '__main__':
#     main()






# Q4. [2 points] Is it possible for A* to be slower than Dijkstra’s at finding a solution? If so, givean example.


# '''

# Yes, A* can be slower than Dijkstra's algorithm in certain scenarios, particularly when the heuristic function is poorly chosen or when the problem structure does not benefit from the extra computation involved in the heuristic. For example, in cases where the heuristic provides little useful information, A* incurs the overhead of computing the heuristic without improving search efficiency. This is often the case in dense graphs with uniform costs, where A* explores a similar number of nodes as Dijkstra's algorithm but with the additional burden of heuristic calculation. In the worst-case scenario, where the heuristic overestimates the cost to the goal, A* may end up exploring the entire graph like Dijkstra’s algorithm but with the added cost of computing the heuristic at each step. Consider a grid-based pathfinding problem where the grid is mostly open, the heuristic function is computationally expensive, and the shortest path is straightforward. In this case, Dijkstra’s algorithm would methodically explore nodes and quickly find the path, while A* would spend extra time computing an unnecessary heuristic. Thus, while A* is typically faster when a good heuristic is available, its performance can degrade if the heuristic is ineffective or computationally expensive.

# example below.

# '''

# # example

# import heapq
# import time
# from math import sqrt

# # Define start and goal
# start = (0, 0)
# goal = (4, 4)

# # grid size
# grid_size = 100
# grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# # obstacles
# for i in range(grid_size // 2):
#     grid[i][grid_size // 2] = 1
#     grid[grid_size // 2][i] = 1

# start = (0, 0)
# goal = (grid_size - 1, grid_size - 1)

# def heuristic(a, b):
#     # More complex heuristic with added computational overhead
#     dx = abs(b[0] - a[0])
#     dy = abs(b[1] - a[1])
#     return sqrt(dx*dx + dy*dy) * (dx*dx + dy*dy)
# def get_neighbors(node):
#     directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
#     result = []
#     for direction in directions:
#         neighbor = (node[0] + direction[0], node[1] + direction[1])
#         if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
#             result.append(neighbor)
#     return result

# def dijkstra(start, goal):
#     heap = [(0, start)]
#     visited = set()
    
#     while heap:
#         (cost, node) = heapq.heappop(heap)
        
#         if node == goal:
#             return cost
        
#         if node in visited:
#             continue
        
#         visited.add(node)
        
#         for neighbor in get_neighbors(node):
#             if neighbor not in visited:
#                 heapq.heappush(heap, (cost + 1, neighbor))
    
#     return float('inf')

# def astar(start, goal):
#     heap = [(0, start)]
#     g_score = {start: 0}
#     visited = set()
    
#     while heap:
#         (f, node) = heapq.heappop(heap)
        
#         if node == goal:
#             return g_score[node]
        
#         if node in visited:
#             continue
        
#         visited.add(node)
        
#         for neighbor in get_neighbors(node):
#             tentative_g = g_score[node] + 1
            
#             if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                 g_score[neighbor] = tentative_g
#                 f = tentative_g + heuristic(neighbor, goal)
#                 heapq.heappush(heap, (f, neighbor))
    
#     return float('inf')

# # Run algorithms multiple times
# num_runs = 10
# dijkstra_total_time = 0
# astar_total_time = 0

# for _ in range(num_runs):
#     start_time = time.perf_counter()
#     dijkstra_cost = dijkstra(start, goal)
#     dijkstra_total_time += time.perf_counter() - start_time

#     start_time = time.perf_counter()
#     astar_cost = astar(start, goal)
#     astar_total_time += time.perf_counter() - start_time

# dijkstra_avg_time = dijkstra_total_time / num_runs
# astar_avg_time = astar_total_time / num_runs

# print(f"Dijkstra's Algorithm: Cost = {dijkstra_cost}, Avg Time = {dijkstra_avg_time:.6f} seconds")
# print(f"A* Algorithm: Cost = {astar_cost}, Avg Time = {astar_avg_time:.6f} seconds")



# Q5. [5 points] In probabilistic road map.py, the code for PRM generation is already completed. Using the PRM, A template has been provided to use Dijkstra’s on the existing PRM to find the solution. Finish the implementation such that a similar output is obtained (Since PRM is probabilistic, the exact output isn’t expected. However, the path between the start and goal must be found)


# import math
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial import KDTree

# # parameter
# N_SAMPLE = 500  # number of sample_points
# N_KNN = 10  # number of edge from one sampled point
# MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

# show_animation = True


# class Node:
#     def __init__(self, x, y, cost, parent_index):
#         self.x = x
#         self.y = y
#         self.cost = cost
#         self.parent_index = parent_index

# def prm_planning(start_x, start_y, goal_x, goal_y,
#                  obstacle_x_list, obstacle_y_list, robot_radius, *, rng=None):
    
#     obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

#     sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
#                                        robot_radius,
#                                        obstacle_x_list, obstacle_y_list,
#                                        obstacle_kd_tree, rng)
#     if show_animation:
#         plt.plot(sample_x, sample_y, ".b")

#     road_map = generate_road_map(sample_x, sample_y,
#                                  robot_radius, obstacle_kd_tree)

#     rx, ry = dijkstra_planning(
#         start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

#     return rx, ry


# def is_collision(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_kd_tree):
#     x = start_x
#     y = start_y
#     dx = goal_x - start_x
#     dy = goal_y - start_y
#     yaw = math.atan2(goal_y - start_y, goal_x - start_x)
#     d = math.hypot(dx, dy)

#     if d >= MAX_EDGE_LEN:
#         return True

#     D = robot_radius
#     n_step = round(d / D)

#     for i in range(n_step):
#         dist, _ = obstacle_kd_tree.query([x, y])
#         if dist <= robot_radius:
#             return True  
#         x += D * math.cos(yaw)
#         y += D * math.sin(yaw)

#     dist, _ = obstacle_kd_tree.query([goal_x, goal_y])
#     if dist <= robot_radius:
#         return True  

#     return False  


# def generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree):

#     road_map = []
#     n_sample = len(sample_x)
#     sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

#     for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

#         dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
#         edge_id = []

#         for ii in range(1, len(indexes)):
#             nx = sample_x[indexes[ii]]
#             ny = sample_y[indexes[ii]]

#             if not is_collision(ix, iy, nx, ny, robot_radius, obstacle_kd_tree):
#                 edge_id.append(indexes[ii])

#             if len(edge_id) >= N_KNN:
#                 break

#         road_map.append(edge_id)

#     return road_map


# def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
#     start_node = Node(start_x, start_y, 0.0, -1)
#     goal_node = Node(goal_x, goal_y, 0.0, -1)

#     open_set, closed_set = dict(), dict()
#     open_set[len(road_map) - 2] = start_node

#     path_found = True

#     while True:
#         # Find the Node in open_set with least cost and assign it to current
#         current_index = min(open_set, key=lambda idx: open_set[idx].cost)
#         current = open_set[current_index]

#         # DO NOT ALTER THE NEXT 6 LINES. 
#         if show_animation and len(closed_set.keys()) % 2 == 0:
#             plt.gcf().canvas.mpl_connect(
#                 'key_release_event',
#                 lambda event: [exit(0) if event.key == 'escape' else None])
#             plt.plot(current.x, current.y, "xg")
#             plt.pause(0.001)

#         # Check if the current Node is the goal node
#         if current_index == len(road_map) - 1:  # Goal is the last node in road_map
#             goal_node.parent_index = current.parent_index
#             goal_node.cost = current.cost
#             break

#         # Remove the current node from open_set and add it to closed_set
#         del open_set[current_index]
#         closed_set[current_index] = current

#         # Expand to neighbors
#         for neighbor_index in road_map[current_index]:
#             if neighbor_index in closed_set:
#                 continue

#             neighbor = Node(sample_x[neighbor_index], sample_y[neighbor_index], 0.0, current_index)
#             cost_to_neighbor = current.cost + math.hypot(current.x - neighbor.x, current.y - neighbor.y)

#             if neighbor_index not in open_set:
#                 neighbor.cost = cost_to_neighbor
#                 open_set[neighbor_index] = neighbor
#             elif cost_to_neighbor < open_set[neighbor_index].cost:
#                 open_set[neighbor_index].cost = cost_to_neighbor
#                 open_set[neighbor_index].parent_index = current_index

#     # DO NOT ALTER THE NEXT 8 LINES
#     rx, ry = [goal_node.x], [goal_node.y]
#     parent_index = goal_node.parent_index
#     while parent_index != -1:
#         n = closed_set[parent_index]
#         rx.append(n.x)
#         ry.append(n.y)
#         parent_index = n.parent_index

#     return rx, ry


# def sample_points(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y, obstacle_kd_tree, rng):
#     max_x = max(obstacle_x)
#     max_y = max(obstacle_y)
#     min_x = min(obstacle_x)
#     min_y = min(obstacle_y)

#     sample_x, sample_y = [], []

#     if rng is None:
#         rng = np.random.default_rng()

#     while len(sample_x) <= N_SAMPLE:
#         tx = (rng.random() * (max_x - min_x)) + min_x
#         ty = (rng.random() * (max_y - min_y)) + min_y

#         dist, index = obstacle_kd_tree.query([tx, ty])

#         if dist >= robot_radius:
#             sample_x.append(tx)
#             sample_y.append(ty)

#     sample_x.append(start_x)
#     sample_y.append(start_y)
#     sample_x.append(goal_x)
#     sample_y.append(goal_y)

#     return sample_x, sample_y


# def main(rng=None):
#     print(__file__ + " start!!")

#     # start and goal position
#     start_x = 10.0  # [m]
#     start_y = 10.0  # [m]
#     goal_x = 50.0  # [m]
#     goal_y = 50.0  # [m]
#     robot_radius = 5.0  # [m]

#     obstacle_x = []
#     obstacle_y = []

#     for i in range(60):
#         obstacle_x.append(i)
#         obstacle_y.append(0.0)
#     for i in range(60):
#         obstacle_x.append(60.0)
#         obstacle_y.append(i)
#     for i in range(61):
#         obstacle_x.append(i)
#         obstacle_y.append(60.0)
#     for i in range(61):
#         obstacle_x.append(0.0)
#         obstacle_y.append(i)
#     for i in range(40):
#         obstacle_x.append(20.0)
#         obstacle_y.append(i)
#     for i in range(40):
#         obstacle_x.append(40.0)
#         obstacle_y.append(60.0 - i)

#     if show_animation:
#         plt.plot(obstacle_x, obstacle_y, ".k")
#         plt.plot(start_x, start_y, "^r")
#         plt.plot(goal_x, goal_y, "^c")
#         plt.grid(True)
#         plt.axis("equal")

#     rx, ry = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius, rng=rng)

#     assert rx, 'Cannot found path'

#     if show_animation:
#         plt.plot(rx, ry, "-r")
#         plt.pause(0.001)
#         plt.show()


# if __name__ == '__main__':
#     main()


# Q6. [5 points] What is the average cost of the solution over 5 trials for the above implementation. What simple parameter change in PRM would result in the reduction of the average cost of solution provided by Dijkstra’s? Demonstrate by changing the appropriate parameter in the code.


# import math
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial import KDTree

# # parameter
# N_SAMPLE = 500  # number of sample_points
# N_KNN = 10  # number of edge from one sampled point
# MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

# show_animation = True


# class Node:
#     def __init__(self, x, y, cost, parent_index):
#         self.x = x
#         self.y = y
#         self.cost = cost
#         self.parent_index = parent_index

# def prm_planning(start_x, start_y, goal_x, goal_y,
#                  obstacle_x_list, obstacle_y_list, robot_radius, *, rng=None):
    
#     obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

#     sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
#                                        robot_radius,
#                                        obstacle_x_list, obstacle_y_list,
#                                        obstacle_kd_tree, rng)
#     if show_animation:
#         plt.plot(sample_x, sample_y, ".b")

#     road_map = generate_road_map(sample_x, sample_y,
#                                  robot_radius, obstacle_kd_tree)

#     rx, ry = dijkstra_planning(
#         start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

#     return rx, ry


# def is_collision(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_kd_tree):
#     x = start_x
#     y = start_y
#     dx = goal_x - start_x
#     dy = goal_y - start_y
#     yaw = math.atan2(goal_y - start_y, goal_x - start_x)
#     d = math.hypot(dx, dy)

#     if d >= MAX_EDGE_LEN:
#         return True

#     D = robot_radius
#     n_step = round(d / D)

#     for i in range(n_step):
#         dist, _ = obstacle_kd_tree.query([x, y])
#         if dist <= robot_radius:
#             return True  
#         x += D * math.cos(yaw)
#         y += D * math.sin(yaw)

#     dist, _ = obstacle_kd_tree.query([goal_x, goal_y])
#     if dist <= robot_radius:
#         return True  

#     return False  


# def generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree):

#     road_map = []
#     n_sample = len(sample_x)
#     sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

#     for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

#         dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
#         edge_id = []

#         for ii in range(1, len(indexes)):
#             nx = sample_x[indexes[ii]]
#             ny = sample_y[indexes[ii]]

#             if not is_collision(ix, iy, nx, ny, robot_radius, obstacle_kd_tree):
#                 edge_id.append(indexes[ii])

#             if len(edge_id) >= N_KNN:
#                 break

#         road_map.append(edge_id)

#     return road_map


# def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
#     start_node = Node(start_x, start_y, 0.0, -1)
#     goal_node = Node(goal_x, goal_y, 0.0, -1)

#     open_set, closed_set = dict(), dict()
#     open_set[len(road_map) - 2] = start_node

#     path_found = True

#     while True:
#         # Find the Node in open_set with least cost and assign it to current
#         current_index = min(open_set, key=lambda idx: open_set[idx].cost)
#         current = open_set[current_index]

#         # DO NOT ALTER THE NEXT 6 LINES. 
#         if show_animation and len(closed_set.keys()) % 2 == 0:
#             plt.gcf().canvas.mpl_connect(
#                 'key_release_event',
#                 lambda event: [exit(0) if event.key == 'escape' else None])
#             plt.plot(current.x, current.y, "xg")
#             plt.pause(0.001)

#         # Check if the current Node is the goal node
#         if current_index == len(road_map) - 1:  # Goal is the last node in road_map
#             goal_node.parent_index = current.parent_index
#             goal_node.cost = current.cost
#             break

#         # Remove the current node from open_set and add it to closed_set
#         del open_set[current_index]
#         closed_set[current_index] = current

#         # Expand to neighbors
#         for neighbor_index in road_map[current_index]:
#             if neighbor_index in closed_set:
#                 continue

#             neighbor = Node(sample_x[neighbor_index], sample_y[neighbor_index], 0.0, current_index)
#             cost_to_neighbor = current.cost + math.hypot(current.x - neighbor.x, current.y - neighbor.y)

#             if neighbor_index not in open_set:
#                 neighbor.cost = cost_to_neighbor
#                 open_set[neighbor_index] = neighbor
#             elif cost_to_neighbor < open_set[neighbor_index].cost:
#                 open_set[neighbor_index].cost = cost_to_neighbor
#                 open_set[neighbor_index].parent_index = current_index

#     # DO NOT ALTER THE NEXT 8 LINES
#     rx, ry = [goal_node.x], [goal_node.y]
#     parent_index = goal_node.parent_index
#     while parent_index != -1:
#         n = closed_set[parent_index]
#         rx.append(n.x)
#         ry.append(n.y)
#         parent_index = n.parent_index

#     return rx, ry


# def sample_points(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y, obstacle_kd_tree, rng):
#     max_x = max(obstacle_x)
#     max_y = max(obstacle_y)
#     min_x = min(obstacle_x)
#     min_y = min(obstacle_y)

#     sample_x, sample_y = [], []

#     if rng is None:
#         rng = np.random.default_rng()

#     while len(sample_x) <= N_SAMPLE:
#         tx = (rng.random() * (max_x - min_x)) + min_x
#         ty = (rng.random() * (max_y - min_y)) + min_y

#         dist, index = obstacle_kd_tree.query([tx, ty])

#         if dist >= robot_radius:
#             sample_x.append(tx)
#             sample_y.append(ty)

#     sample_x.append(start_x)
#     sample_y.append(start_y)
#     sample_x.append(goal_x)
#     sample_y.append(goal_y)

#     return sample_x, sample_y



# def main(num_trials=5):
#     total_cost = 0
    
#     for trial in range(num_trials):
#         print(f"Trial {trial + 1}")
        
#         # start and goal position
#         start_x, start_y = 10.0, 10.0  # [m]
#         goal_x, goal_y = 50.0, 50.0  # [m]
#         robot_radius = 5.0  # [m]

#         obstacle_x, obstacle_y = create_obstacles()

#         if show_animation:
#             plt.clf()
#             plt.plot(obstacle_x, obstacle_y, ".k")
#             plt.plot(start_x, start_y, "^r")
#             plt.plot(goal_x, goal_y, "^c")
#             plt.grid(True)
#             plt.axis("equal")

#         rx, ry = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius)

#         assert rx, 'Cannot find path'

#         path_cost = calculate_path_cost(rx, ry)
#         total_cost += path_cost

#         if show_animation:
#             plt.plot(rx, ry, "-r")
#             plt.pause(0.001)
#             plt.show()

#     average_cost = total_cost / num_trials
#     print(f"Average cost over {num_trials} trials: {average_cost:.2f}")

# def calculate_path_cost(rx, ry):
#     cost = 0
#     for i in range(len(rx) - 1):
#         cost += math.hypot(rx[i+1] - rx[i], ry[i+1] - ry[i])
#     return cost

# if __name__ == '__main__':
#     main(5)


# def main(num_trials=5):
#     total_cost = 0
    
#     for trial in range(num_trials):
#         print(f"Trial {trial + 1}")
        
#         # start and goal position
#         start_x, start_y = 10.0, 10.0  # [m]
#         goal_x, goal_y = 50.0, 50.0  # [m]
#         robot_radius = 5.0  # [m]

#         # Create obstacles
#         obstacle_x = []
#         obstacle_y = []

#         for i in range(60):
#             obstacle_x.append(i)
#             obstacle_y.append(0.0)
#         for i in range(60):
#             obstacle_x.append(60.0)
#             obstacle_y.append(i)
#         for i in range(61):
#             obstacle_x.append(i)
#             obstacle_y.append(60.0)
#         for i in range(61):
#             obstacle_x.append(0.0)
#             obstacle_y.append(i)
#         for i in range(40):
#             obstacle_x.append(20.0)
#             obstacle_y.append(i)
#         for i in range(40):
#             obstacle_x.append(40.0)
#             obstacle_y.append(60.0 - i)

#         if show_animation:
#             plt.clf()
#             plt.plot(obstacle_x, obstacle_y, ".k")
#             plt.plot(start_x, start_y, "^r")
#             plt.plot(goal_x, goal_y, "^c")
#             plt.grid(True)
#             plt.axis("equal")

#         rx, ry = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius)

#         assert rx, 'Cannot find path'

#         path_cost = calculate_path_cost(rx, ry)
#         total_cost += path_cost

#         if show_animation:
#             plt.plot(rx, ry, "-r")
#             plt.pause(0.001)
#             plt.show()

#     average_cost = total_cost / num_trials
#     print(f"Average cost over {num_trials} trials with N_SAMPLE = {N_SAMPLE}: {average_cost:.2f}")

# def calculate_path_cost(rx, ry):
#     cost = 0
#     for i in range(len(rx) - 1):
#         cost += math.hypot(rx[i+1] - rx[i], ry[i+1] - ry[i])
#     return cost

# # Make sure to include all other necessary functions here (prm_planning, sample_points, etc.)

# if __name__ == '__main__':
#     main(5)


# Q7. [10 points] A template has been provided to implement and visualize RRT algorithm via rrt.py. A brief psuedocode has been commented in the python file to help in the implementation. Finish the implementation such that a similar output is obtained (Since RRT is probabilistic, the exact output isn’t expected. However, the path between the start and goal must be found)


import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:

    class Node:

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            if random.randint(0, 100) > self.goal_sample_rate:
                rnd_node = self.Node(
                    random.uniform(self.min_rand, self.max_rand),
                    random.uniform(self.min_rand, self.max_rand))
            else:
                rnd_node = self.Node(self.end.x, self.end.y)

            nearest_node = self.get_nearest_node_index(self.node_list, rnd_node)
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return node_list[minind]

    @staticmethod
    def check_collision(node, obstacleList):
        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # collision

        return True  # safe

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(goal_x=6.0, goal_y=10.0):
    print("start " + __file__)
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    rrt = RRT(
        start=[0, 0],
        goal=[goal_x, goal_y],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        robot_radius=0.8
        )
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01) 
            plt.show()

if __name__ == '__main__':
    main()


# Q8. [3 points] Does the above implementation work if the value of expand dis is changed from 3.0 to 0.1? Why or why not? If not, what parameter can be changed to ensure the implementation works with expand dis being 0.1?

'''

The `expand_dis` parameter in the RRT algorithm controls the maximum distance the tree can expand in a single step. Reducing it from 3.0 to 0.1 will drastically alter the algorithm’s behavior. The tree will grow more slowly, with each new node only 0.1 units away from its parent, compared to the previous 3.0 units. This will make it difficult for the algorithm to reach the goal, as the condition for connecting to the goal will rarely be met. Additionally, the collision detection will be overly fine-grained due to shorter path segments being checked. As a result, the algorithm will require many more iterations to reach the goal, possibly hitting the maximum iteration limit without finding a solution. To improve performance with such a small `expand_dis`, it's essential to adjust parameters like `path_resolution`, reducing it from 0.5 to 0.05, and increasing `max_iter` to 10,000 to allow for more iterations. While these changes will make the algorithm functional with the smaller expansion step, the trade-offs include slower performance, a more jagged path, and a higher likelihood of getting stuck in complex environments. If performance remains unsatisfactory, using RRT* or adjusting parameters like `goal_sample_rate` to bias the search towards the goal might offer better results.

'''

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:

    class Node:

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            if random.randint(0, 100) > self.goal_sample_rate:
                rnd_node = self.Node(
                    random.uniform(self.min_rand, self.max_rand),
                    random.uniform(self.min_rand, self.max_rand))
            else:
                rnd_node = self.Node(self.end.x, self.end.y)

            nearest_node = self.get_nearest_node_index(self.node_list, rnd_node)
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return node_list[minind]

    @staticmethod
    def check_collision(node, obstacleList):
        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # collision

        return True  # safe

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(goal_x=6.0, goal_y=10.0):
    print("start " + __file__)
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    rrt = RRT(
        start=[0, 0],
        goal=[goal_x, goal_y],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        robot_radius=0.8,
        expand_dis=0.1,
        path_resolution=0.05,
        max_iter=10000  # Increased to allow more iterations
        )
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01) 
            plt.show()


if __name__ == '__main__':
    main()
