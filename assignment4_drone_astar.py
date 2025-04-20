import heapq
import time

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(pos, grid):
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for dx, dy in dirs:
        nx, ny = pos[0] + dx, pos[1] + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != "#":
            neighbors.append((nx, ny))
    return neighbors

def a_star(grid, start, goal):
    open_set = [(0, start)]
    came_from = {}
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    visited = set()
    node_count = 1

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct(came_from, start, goal), node_count

        visited.add(current)

        for neighbor in get_neighbors(current, grid):
            temp_g = g[current] + 1
            if neighbor not in g or temp_g < g[neighbor]:
                came_from[neighbor] = current
                g[neighbor] = temp_g
                f[neighbor] = temp_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f[neighbor], neighbor))
                node_count += 1

    return None, node_count

def gbfs(grid, start, goal):
    open_set = [(heuristic(start, goal), start)]
    came_from = {}
    visited = {start}
    node_count = 1

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct(came_from, start, goal), node_count

        for neighbor in get_neighbors(current, grid):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                heapq.heappush(open_set, (heuristic(neighbor, goal), neighbor))
                node_count += 1

    return None, node_count

def reconstruct(came_from, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path

def print_grid(grid, path):
    grid_copy = [row[:] for row in grid]
    for x, y in path:
        if grid_copy[x][y] not in "SG":
            grid_copy[x][y] = "*"
    for row in grid_copy:
        print("".join(row))

def find(grid, symbol):
    for i, row in enumerate(grid):
        for j, val in enumerate(row):
            if val == symbol:
                return (i, j)

def run_assignment4():
    grid = [
        list("S123#"),
        list("23459"),
        list("12345"),
        list("45678"),
        list("9#876"),
        list("9876G")
    ]
    start = find(grid, "S")
    goal = find(grid, "G")

    # GBFS
    t0 = time.perf_counter()
    gbfs_path, gbfs_node_count = gbfs(grid, start, goal)
    t1 = time.perf_counter()
    gbfs_time = (t1 - t0) * 1000

    # A*
    t0 = time.perf_counter()
    a_star_path, a_star_node_count = a_star(grid, start, goal)
    t1 = time.perf_counter()
    a_star_time = (t1 - t0) * 1000

    print("\nAssignment 4 - GBFS Path (Drone Navigation):")
    if gbfs_path:
        print_grid(grid, gbfs_path)
    else:
        print("No path found.")
    print(f"GBFS Execution Time: {gbfs_time:.4f} ms")
    print(f"GBFS Nodes Explored: {gbfs_node_count}")

    print("\nAssignment 4 - A* Path (Drone Navigation):")
    if a_star_path:
        print_grid(grid, a_star_path)
    else:
        print("No path found.")
    print(f"A* Execution Time: {a_star_time:.4f} ms")
    print(f"A* Nodes Explored: {a_star_node_count}")
