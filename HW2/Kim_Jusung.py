"""

Grid based Dijkstra planning

COEN 5830 HW

Full Name: Jusung Kim

Coll
"""
import heapq
import time
from math import sqrt

# Increase grid size
grid_size = 100
grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# Add some obstacles
for i in range(grid_size // 2):
    grid[i][grid_size // 2] = 1
    grid[grid_size // 2][i] = 1

start = (0, 0)
goal = (grid_size - 1, grid_size - 1)

def heuristic(a, b):
    # More complex heuristic with added computational overhead
    dx = abs(b[0] - a[0])
    dy = abs(b[1] - a[1])
    return sqrt(dx*dx + dy*dy) * (dx*dx + dy*dy)


def get_neighbors(node):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    result = []
    for direction in directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1])
        if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
            result.append(neighbor)
    return result

def dijkstra(start, goal):
    heap = [(0, start)]
    visited = set()
    
    while heap:
        (cost, node) = heapq.heappop(heap)
        
        if node == goal:
            return cost
        
        if node in visited:
            continue
        
        visited.add(node)
        
        for neighbor in get_neighbors(node):
            if neighbor not in visited:
                heapq.heappush(heap, (cost + 1, neighbor))
    
    return float('inf')

def astar(start, goal):
    heap = [(0, start)]
    g_score = {start: 0}
    visited = set()
    
    while heap:
        (f, node) = heapq.heappop(heap)
        
        if node == goal:
            return g_score[node]
        
        if node in visited:
            continue
        
        visited.add(node)
        
        for neighbor in get_neighbors(node):
            tentative_g = g_score[node] + 1
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(heap, (f, neighbor))
    
    return float('inf')

# Run algorithms multiple times
num_runs = 10
dijkstra_total_time = 0
astar_total_time = 0

for _ in range(num_runs):
    start_time = time.perf_counter()
    dijkstra_cost = dijkstra(start, goal)
    dijkstra_total_time += time.perf_counter() - start_time

    start_time = time.perf_counter()
    astar_cost = astar(start, goal)
    astar_total_time += time.perf_counter() - start_time

dijkstra_avg_time = dijkstra_total_time / num_runs
astar_avg_time = astar_total_time / num_runs

print(f"Dijkstra's Algorithm: Cost = {dijkstra_cost}, Avg Time = {dijkstra_avg_time:.6f} seconds")
print(f"A* Algorithm: Cost = {astar_cost}, Avg Time = {astar_avg_time:.6f} seconds")