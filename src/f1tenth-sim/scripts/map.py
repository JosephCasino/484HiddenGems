#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from collections import deque

current_position = None
path_pub = None

def odom_callback(msg):
    global current_position
    pose = msg.pose.pose.position
    current_position = (pose.x, pose.y)

# Convert the grid to a lower resolution to improve computing time
def process_grid(grid, factor):
    h, w = grid.shape
    new_h = h // factor
    new_w = w // factor
    downsampled = np.full((new_h, new_w), -1, dtype=np.int8)

    for i in range(new_h):
        for j in range(new_w):
            block = grid[i*factor:(i+1)*factor, j*factor:(j+1)*factor]
            if np.any(block == 1):
                downsampled[i, j] = 1
            elif np.all(block == 0):
                downsampled[i, j] = 0
            else:
                downsampled[i, j] = -1

    return downsampled

# Getting neighbors of a node using the matrix as a graph
def neighbors(grid, i, j):
    H, W = grid.shape
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        ni, nj = i + dx, j + dy
        if 0 <= ni < H and 0 <= nj < W:
            yield ni, nj

#distance hueristic functions
def distance_func(grid):
    H, W = grid.shape
    distance = np.full((H, W), np.inf)
    queue = deque()

    for i in range(H):
        for j in range(W):
            if grid[i, j] == 1:
                distance[i, j] = 0
                queue.append((i, j))

    while queue:
        i, j = queue.popleft()
        for ni, nj in neighbors(grid, i, j):
            if distance[ni, nj] > distance[i, j] + 1 and grid[ni, nj] != 1:
                distance[ni, nj] = distance[i, j] + 1
                queue.append((ni, nj))

    return distance

def bfs_search(grid, start, distance_map):
    visited = set()
    queue = deque([(start, [start])])
    visited.add(start)

    while queue:
        queue = deque(sorted(queue, key=lambda x: -distance_map[x[0][0], x[0][1]]))
        (i, j), path = queue.popleft()

        if grid[i, j] == -1:
            return path

        for ni, nj in neighbors(grid, i, j):
            if grid[ni, nj] != 1 and (ni, nj) not in visited:
                visited.add((ni, nj))
                queue.append(((ni, nj), path + [(ni, nj)]))

    return None

def grid_to_world(i, j, factor, resolution, origin):
    y = origin.position.y + resolution * factor * (i + 0.5)
    x = origin.position.x + resolution * factor * (j + 0.5)
    return x, y

def map_callback(msg):
    global current_position, path_pub
    if current_position is None:
        print("Waiting for odometry...")
        return

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin

    data = np.array(msg.data).reshape((height, width))
    grid = np.where(data == 100, 1, 0)
    grid[data == -1] = -1

    factor = 4
    grid = process_grid(grid, factor)

    map_x = (current_position[0] - origin.position.x) / resolution
    map_y = (current_position[1] - origin.position.y) / resolution

    map_x = int(map_x)
    map_y = int(map_y)

    start_i = map_y // factor
    start_j = map_x // factor

    if not (0 <= start_i < grid.shape[0] and 0 <= start_j < grid.shape[1]):
        print("Start position out of bounds.")
        return

    if grid[start_i, start_j] == 1:
        print("Start position is occupied.")
        return

    distance_map = distance_func(grid)
    path = bfs_search(grid, (start_i, start_j), distance_map)

    if path:
        print(f"Found path of length {len(path)}")

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for i, j in path:
            x, y = grid_to_world(i, j, factor, resolution, origin)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            if grid[i, j] == 0:
                grid[i, j] = 2

        path_pub.publish(path_msg)
    else:
        print("No reachable unknown nodes found.")
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = []
        path_pub.publish(path_msg)

    if grid[start_i, start_j] != 1:
        grid[start_i, start_j] = 3

    colors = ['gray', 'white', 'black', 'red', 'green']
    cmap = ListedColormap(colors)
    bounds = [-2, -0.5, 0.5, 1.5, 2.5, 3.5]
    norm = BoundaryNorm(bounds, cmap.N)

    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap=cmap, norm=norm, origin='lower')
    plt.title("Occupancy Grid")
    plt.savefig("map.png", dpi=300)
    plt.close()

def main():
    global path_pub
    rospy.init_node("map_listener")
    rospy.Subscriber("/car_1/base/odom", Odometry, odom_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    path_pub = rospy.Publisher("/planned_path", Path, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()