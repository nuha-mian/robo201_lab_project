import yaml
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
import time

def yaml_to_occupancy_grid(yaml_file):
    # Load YAML 
    with open(yaml_file, 'r') as f:
        # save yaml details in a dictionary
        map_metadata = yaml.safe_load(f)
    
    #get the image and thresholds
    image_path = map_metadata["image"]
    occ_thresh = map_metadata["occupied_thresh"]
    free_thresh = map_metadata["free_thresh"]

    # Load PGM/PNG image as grayscale
    img = Image.open(image_path).convert('L')  # 0-255
    img = np.array(img)

    # Normalized values from 0.0 to 1.0
    normalized = img / 255.0

    grid = np.zeros_like(normalized, dtype=int)


    grid[normalized < occ_thresh] = 1      # obstacle
    grid[normalized > free_thresh] = 0     # free
    mask_unknown = (normalized >= occ_thresh) & (normalized <= free_thresh)
    grid[mask_unknown] = -1                # unknown

    return grid


def is_valid_node(node:tuple,grid):

    (row,col) = node
    total_rows = 38
    total_cols = 30

    if row >= 0 and row < total_rows:
        if col >= 0 and col < total_cols:
            if grid[row][col] == 0:
                return True
    return False


def neighbors(node:tuple):

    (row,col) = node

    top_node = (row-1,col)
    bottom_node = (row+1,col)
    right_node = (row,col+1)
    left_node = (row,col-1)

    return [top_node,bottom_node,left_node,right_node]


def heuristic(node1,node2):

    (x1,y1) = node1
    (x2,y2) = node2
    manhattan_distance = abs(x1-x2)+abs(y1-y2)
    return manhattan_distance


def BFS(start_node,goal_node,grid):

        open_list = PriorityQueue()
        open_list.put((0,start_node))

        closed_list = set() 

        parents = {start_node:None} 

        cost_from_start_node = {start_node:0}

        while not open_list.empty():

            (current_cost,current_node) = open_list.get()

            if current_node == goal_node:
                break
            if current_node in closed_list:
                continue 
            else:
                closed_list.add(current_node)

            for next_node in neighbors(current_node): 
                if is_valid_node(next_node,grid) and next_node not in closed_list:
                    new_cost = current_cost+1 
                    if next_node not in cost_from_start_node or new_cost < cost_from_start_node[next_node]:
                        cost_from_start_node[next_node] = new_cost
                        parents[next_node] = current_node
                        priority_cost = heuristic(next_node,goal_node) 
                        open_list.put((priority_cost,next_node))


        path = []
        current_node = goal_node
        while current_node != start_node:
            path.append(current_node)
            current_node = parents.get(current_node)
        path.append(start_node)
        path.reverse()
        total_exapnded_nodes = len(closed_list)
        return total_exapnded_nodes , path , cost_from_start_node[goal_node]

    
def plot_grid(grid, start_node, goal_node, path):
    rows, cols = grid.shape

    fig, ax = plt.subplots(figsize=(10, 10))

    ax.imshow(grid, cmap="gray_r", origin="lower")

    # Add grid lines

    ax.set_xticks(np.arange(-0.5, cols, 1))
    ax.set_yticks(np.arange(-0.5, rows, 1))


    ax.grid(which="both", color="lightgray", linewidth=0.5)

    # Extract path coordinates
    x_val = [c for (r, c) in path]
    y_val = [r for (r, c) in path]

    # Plot start and goal
    ax.scatter(start_node[1], start_node[0], marker="o", color="blue", label="start")
    ax.scatter(goal_node[1], goal_node[0], marker="o", color="yellow", label="goal")

    # Plot path
    ax.plot(x_val, y_val, color="green", linewidth=2, label="path")

    ax.set_title(" Best-First Search Algorithm")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()

    plt.show()

def inflate_grid(grid, inflation_cells):
    rows, cols = grid.shape
    inflated = grid.copy()

    # Find all obstacle cells
    obstacle_indices = np.argwhere(grid == 1)

    for (r, c) in obstacle_indices:
        # Inflate around each obstacle
        for dr in range(-inflation_cells, inflation_cells + 1):
            for dc in range(-inflation_cells, inflation_cells + 1):

                # Check if inside circle inflation radius
                if dr*dr + dc*dc <= inflation_cells*inflation_cells:
                    rr = r + dr
                    cc = c + dc

                    if 0 <= rr < rows and 0 <= cc < cols:
                        inflated[rr, cc] = 1

    return inflated



if __name__ == '__main__':
    
    grid = yaml_to_occupancy_grid("my_robot_map_real.yaml")
    resolution = 0.05  # read this from yaml file
    robot_radius = 0.105   # meters
    inflation_cells = int(robot_radius // resolution) -1 

    inflated_grid = inflate_grid(grid, inflation_cells)
    
    
    goal_node = (25, 17)
    start_node = (9,17)
    
    start_time = time.time()
    total_exapnded_nodes , path, cost = BFS(start_node, goal_node, inflated_grid)


    end_time = time.time()
    total_time = (end_time - start_time ) *100
    print("shortest path:",path)
    print(f"run time: {total_time} ms") 
    print("path cost:", len(path))
    print("toatl expanded nodes", total_exapnded_nodes)
    plot_grid(inflated_grid,start_node,goal_node,path)
    