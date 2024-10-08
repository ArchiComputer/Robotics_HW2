"""

Rapidly Exploring Random Tree (RRT) Planner

COEN 5830

Full Name: ADD YOUR NAME HERE
"""

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


# def main(goal_x=6.0, goal_y=10.0):
#     print("start " + __file__)
#     obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
#                     (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
#     rrt = RRT(
#         start=[0, 0],
#         goal=[goal_x, goal_y],
#         rand_area=[-2, 15],
#         obstacle_list=obstacleList,
#         robot_radius=0.8
#         )
#     path = rrt.planning(animation=show_animation)

#     if path is None:
#         print("Cannot find path")
#     else:
#         print("found path!!")
#         if show_animation:
#             rrt.draw_graph()
#             plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
#             plt.grid(True)
#             plt.pause(0.01) 
#             plt.show()

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


# Let's analyze the impact of changing the `expand_dis` from 3.0 to 0.1 and consider if the implementation will work correctly.



# The `expand_dis` parameter in the RRT algorithm determines the maximum distance the tree can expand in a single step. Changing it from 3.0 to 0.1 will significantly affect the algorithm's behavior. Let's consider the implications:

# 1. Tree Growth:
#    With a smaller `expand_dis`, the tree will grow much more slowly. Each new node will be at most 0.1 units away from its parent, compared to 3.0 units before.

# 2. Goal Reaching:
#    The condition `if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:` in the `planning` method will rarely be true. This means the algorithm will struggle to connect to the goal even when it's close.

# 3. Collision Checking:
#    The `check_collision` method will be checking very short path segments, which might lead to unnecessarily fine-grained collision detection.

# 4. Performance:
#    The algorithm will likely need many more iterations to reach the goal, potentially hitting the `max_iter` limit before finding a solution.

# The implementation will still run, but it may not be effective or efficient with `expand_dis` set to 0.1. To make it work better with this small `expand_dis`, we should consider changing the following parameter:

# `path_resolution`: This parameter determines the discretization of the path between nodes. Currently, it's set to 0.5, which is larger than the new `expand_dis`. We should reduce `path_resolution` to be smaller than `expand_dis`, for example, 0.05.

# Here's how we can modify the RRT initialization in the `main` function to accommodate these changes:

# Key changes:
# 1. Set `expand_dis=0.1` as requested.
# 2. Reduced `path_resolution` to 0.05, which is half of the new `expand_dis`.
# 3. Increased `max_iter` to 10000 to allow more iterations, as the algorithm will need more steps to reach the goal with smaller expansions.

# With these modifications, the RRT implementation should work with the smaller `expand_dis` value. However, keep in mind that:

# 1. The algorithm will run slower due to the increased number of iterations and nodes.
# 2. The resulting path may be more jagged and less direct than with a larger `expand_dis`.
# 3. The algorithm might be more susceptible to getting stuck in complex environments.

# If you find that the performance is still not satisfactory, you might want to consider using a variant of RRT, such as RRT* (RRT-star), which can improve path quality over time, or adjusting other parameters like `goal_sample_rate` to bias the search more strongly towards the goal.

# Would you like me to explain any of these concepts further or discuss alternative RRT variants that might work better with small expansion distances?