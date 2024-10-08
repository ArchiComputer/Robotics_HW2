import heapq
import time
from typing import List, Tuple, Dict, Set

# Define a larger, more complex grid
grid = [
    "S........................#####",
    "########.##########.....######",
    "#......#.#........#.....#....#",
    "#.####.#.#.######.#.....#.##.#",
    "#.#.....#.#......#.#######.#.#",
    "#.#.#####.######.#.........#.#",
    "#.#.....#........#####.###.#.#",
    "#.#####.############.#.#.#.#.#",
    "#.....#..............#.#.#.#.#",
    "#####.##################.#.#.#",
    "#.....#..............#...#.#.#",
    "#.#####.##############.###.#.#",
    "#.#.....#..........#...#...#.#",
    "#.#.#####.########.#.###.###.#",
    "#.#.......#......#.#.#.....#.#",
    "#.#########.####.#.#.#####.#.#",
    "#...........#....#.#.......#.#",
    "###########.#.####.#########.#",
    "#...........#..............#G#",
    "##############################"
]

def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def get_neighbors(pos: Tuple[int, int]) -> List[Tuple[int, int]]:
    row, col = pos
    neighbors = [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]
    return [(r, c) for r, c in neighbors if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != '#']

def dijkstra(start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[int, List[Tuple[int, int]], float]:
    start_time = time.time()
    heap = [(0, start)]
    visited = set()
    g_score = {start: 0}
    parents = {}
    nodes_explored = 0

    while heap:
        current_g, current = heapq.heappop(heap)
        nodes_explored += 1

        if current == goal:
            end_time = time.time()
            return nodes_explored, reconstruct_path(parents, start, goal), end_time - start_time

        if current in visited:
            continue

        visited.add(current)

        for neighbor in get_neighbors(current):
            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get(neighbor, float('inf')):
                parents[neighbor] = current
                g_score[neighbor] = tentative_g
                heapq.heappush(heap, (tentative_g, neighbor))

    end_time = time.time()
    return nodes_explored, None, end_time - start_time

def a_star(start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[int, List[Tuple[int, int]], float]:
    start_time = time.time()
    heap = [(0, start)]
    visited = set()
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    parents = {}
    nodes_explored = 0

    while heap:
        current_f, current = heapq.heappop(heap)
        nodes_explored += 1

        if current == goal:
            end_time = time.time()
            return nodes_explored, reconstruct_path(parents, start, goal), end_time - start_time

        if current in visited:
            continue

        visited.add(current)

        for neighbor in get_neighbors(current):
            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get(neighbor, float('inf')):
                parents[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(heap, (f_score[neighbor], neighbor))

    end_time = time.time()
    return nodes_explored, None, end_time - start_time

def reconstruct_path(parents: Dict[Tuple[int, int], Tuple[int, int]], start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = parents[current]
    path.append(start)
    return path[::-1]

def find_start_and_goal() -> Tuple[Tuple[int, int], Tuple[int, int]]:
    start, goal = None, None
    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            if cell == 'S':
                start = (i, j)
            elif cell == 'G':
                goal = (i, j)
    return start, goal

def print_path_on_grid(path: List[Tuple[int, int]]) -> None:
    grid_copy = [list(row) for row in grid]
    for r, c in path:
        if grid_copy[r][c] not in 'SG':
            grid_copy[r][c] = '*'
    for row in grid_copy:
        print(''.join(row))

def main():
    start, goal = find_start_and_goal()
    
    dijkstra_explored, dijkstra_path, dijkstra_time = dijkstra(start, goal)
    astar_explored, astar_path, astar_time = a_star(start, goal)
    
    print(f"Dijkstra's Algorithm:")
    print(f"  Explored {dijkstra_explored} nodes")
    print(f"  Time taken: {dijkstra_time:.6f} seconds")
    if dijkstra_path:
        print(f"  Path length: {len(dijkstra_path)}")
        print("\nDijkstra's path:")
        print_path_on_grid(dijkstra_path)
    else:
        print("  Couldn't find a path.")
    
    print(f"\nA* Algorithm:")
    print(f"  Explored {astar_explored} nodes")
    print(f"  Time taken: {astar_time:.6f} seconds")
    if astar_path:
        print(f"  Path length: {len(astar_path)}")
        print("\nA* path:")
        print_path_on_grid(astar_path)
    else:
        print("  Couldn't find a path.")

if __name__ == "__main__":
    main()