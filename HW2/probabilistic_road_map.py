"""

Probabilistic Road Map (PRM) Planner

COEN 5830

Full Name: ADD YOUR NAME HERE
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# parameter
N_SAMPLE = 500  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = True


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius, *, rng=None):
    
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius,
                                       obstacle_x_list, obstacle_y_list,
                                       obstacle_kd_tree, rng)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_road_map(sample_x, sample_y,
                                 robot_radius, obstacle_kd_tree)

    rx, ry = dijkstra_planning(
        start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_kd_tree):
    x = start_x
    y = start_y
    dx = goal_x - start_x
    dy = goal_y - start_y
    yaw = math.atan2(goal_y - start_y, goal_x - start_x)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True

    D = robot_radius
    n_step = round(d / D)

    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= robot_radius:
            return True  
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    dist, _ = obstacle_kd_tree.query([goal_x, goal_y])
    if dist <= robot_radius:
        return True  

    return False  


def generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree):

    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, robot_radius, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    return road_map


# def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
#     start_node = Node(start_x, start_y, 0.0, -1)
#     goal_node = Node(goal_x, goal_y, 0.0, -1)

#     open_set, closed_set = dict(), dict()
#     open_set[len(road_map) - 2] = start_node

#     path_found = True

#     while True:
#         #Find the Node in open_set with least cost and assign it to current
#         current = None #Replace None with code to assign the least costly node in open_set
#         #DO NOT ALTER THE NEXT 6 LINES. 
#         if show_animation and len(closed_set.keys()) % 2 == 0:
#             plt.gcf().canvas.mpl_connect(
#                 'key_release_event',
#                 lambda event: [exit(0) if event.key == 'escape' else None])
#             plt.plot(current.x, current.y, "xg")
#             plt.pause(0.001)
#         #If the cost of of current node is 1 less than the length of the PRM, we have reached the goal! If so, assign appropriate values to goal_node.parent_index and goal_node.cost and break from the while loop
#         # Check if the current Node is the goal node. If so, assign appropriate values to goal_node.parent_index and goal_node.cost and break from the while loop
#         # If the current node is not the goal, remove the item from the open set and add it to the closed set
#         # Use the motion model to expand the search to other neighbors in the grid.
#         # Check if the neighbouring cell is a part of closed set. If so, move on.
#         # If the neighboring cell is not within bounds of the state space, move on.
#         # If the neighboring cell is neither in open_set or closed_set, add it to the open set.
#         # If the neighboring cell is a part of the open cell, will expanding from the current node reduce the total cost to reach the neighbor? If so, replace it with the current node. (Essentially changing its parent and cost). 
#     #DO NOT ALTER THE NEXT 8 LINES
#     rx, ry = [goal_node.x], [goal_node.y]
#     parent_index = goal_node.parent_index
#     while parent_index != -1:
#         n = closed_set[parent_index]
#         rx.append(n.x)
#         ry.append(n.y)
#         parent_index = n.parent_index

#     return rx, ry


# def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
#     start_node = Node(start_x, start_y, 0.0, -1)
#     goal_node = Node(goal_x, goal_y, 0.0, -1)

#     open_set, closed_set = dict(), dict()
#     open_set[len(road_map) - 2] = start_node

#     while True:
#         # Find the Node in open_set with least cost and assign it to current
#         current = min(open_set.items(), key=lambda item: item[1].cost)[1]
#         current_index = [k for k, v in open_set.items() if v == current][0]

#         # If the current Node is the goal node
#         if math.hypot(current.x - goal_x, current.y - goal_y) <= robot_radius:
#             goal_node.parent_index = current_index
#             goal_node.cost = current.cost
#             break
        
#         # If the current node is not the goal, remove it from open_set and add to closed_set
#         del open_set[current_index]
#         closed_set[current_index] = current

#         # Expand to neighbors
#         for neighbor_index in road_map[current_index]:
#             if neighbor_index in closed_set:
#                 continue
            
#             neighbor = Node(sample_x[neighbor_index], sample_y[neighbor_index], 0.0, current_index)
#             cost_to_neighbor = current.cost + math.hypot(current.x - neighbor.x, current.y - neighbor.y)

#             if neighbor_index not in open_set or cost_to_neighbor < open_set[neighbor_index].cost:
#                 neighbor.cost = cost_to_neighbor
#                 open_set[neighbor_index] = neighbor

#     # Reconstruct path
#     rx, ry = [goal_node.x], [goal_node.y]
#     parent_index = goal_node.parent_index
#     while parent_index != -1:
#         n = closed_set[parent_index]
#         rx.append(n.x)
#         ry.append(n.y)
#         parent_index = n.parent_index

#     return rx, ry

# def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y, robot_radius):
#     start_node = Node(start_x, start_y, 0.0, -1)
#     goal_node = Node(goal_x, goal_y, 0.0, -1)

#     open_set, closed_set = dict(), dict()
#     open_set[len(road_map) - 2] = start_node

#     path_found = True

#     while True:
#         # Find the Node in open_set with least cost and assign it to current
#         current_index = min(open_set, key=lambda idx: open_set[idx].cost)
#         current = open_set[current_index]

#         # Check if the current node is the goal
#         if math.hypot(current.x - goal_x, current.y - goal_y) <= robot_radius:
#             goal_node.parent_index = current_index
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

#             # If neighbor is not in open_set or a cheaper path is found, add/update the neighbor
#             if neighbor_index not in open_set or cost_to_neighbor < open_set[neighbor_index].cost:
#                 neighbor.cost = cost_to_neighbor
#                 open_set[neighbor_index] = neighbor

#     # Reconstruct the path
#     rx, ry = [goal_node.x], [goal_node.y]
#     parent_index = goal_node.parent_index
#     while parent_index != -1:
#         n = closed_set[parent_index]
#         rx.append(n.x)
#         ry.append(n.y)
#         parent_index = n.parent_index

#     return rx, ry


def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        # Find the Node in open_set with least cost and assign it to current
        current_index = min(open_set, key=lambda idx: open_set[idx].cost)
        current = open_set[current_index]

        # DO NOT ALTER THE NEXT 6 LINES. 
        if show_animation and len(closed_set.keys()) % 2 == 0:
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        # Check if the current Node is the goal node
        if current_index == len(road_map) - 1:  # Goal is the last node in road_map
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the current node from open_set and add it to closed_set
        del open_set[current_index]
        closed_set[current_index] = current

        # Expand to neighbors
        for neighbor_index in road_map[current_index]:
            if neighbor_index in closed_set:
                continue

            neighbor = Node(sample_x[neighbor_index], sample_y[neighbor_index], 0.0, current_index)
            cost_to_neighbor = current.cost + math.hypot(current.x - neighbor.x, current.y - neighbor.y)

            if neighbor_index not in open_set:
                neighbor.cost = cost_to_neighbor
                open_set[neighbor_index] = neighbor
            elif cost_to_neighbor < open_set[neighbor_index].cost:
                open_set[neighbor_index].cost = cost_to_neighbor
                open_set[neighbor_index].parent_index = current_index

    # DO NOT ALTER THE NEXT 8 LINES
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def sample_points(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y, obstacle_kd_tree, rng):
    max_x = max(obstacle_x)
    max_y = max(obstacle_y)
    min_x = min(obstacle_x)
    min_y = min(obstacle_y)

    sample_x, sample_y = [], []

    if rng is None:
        rng = np.random.default_rng()

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        dist, index = obstacle_kd_tree.query([tx, ty])

        if dist >= robot_radius:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(start_x)
    sample_y.append(start_y)
    sample_x.append(goal_x)
    sample_y.append(goal_y)

    return sample_x, sample_y


def main(rng=None):
    print(__file__ + " start!!")

    # start and goal position
    start_x = 10.0  # [m]
    start_y = 10.0  # [m]
    goal_x = 50.0  # [m]
    goal_y = 50.0  # [m]
    robot_radius = 5.0  # [m]

    obstacle_x = []
    obstacle_y = []

    for i in range(60):
        obstacle_x.append(i)
        obstacle_y.append(0.0)
    for i in range(60):
        obstacle_x.append(60.0)
        obstacle_y.append(i)
    for i in range(61):
        obstacle_x.append(i)
        obstacle_y.append(60.0)
    for i in range(61):
        obstacle_x.append(0.0)
        obstacle_y.append(i)
    for i in range(40):
        obstacle_x.append(20.0)
        obstacle_y.append(i)
    for i in range(40):
        obstacle_x.append(40.0)
        obstacle_y.append(60.0 - i)

    if show_animation:
        plt.plot(obstacle_x, obstacle_y, ".k")
        plt.plot(start_x, start_y, "^r")
        plt.plot(goal_x, goal_y, "^c")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius, rng=rng)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
