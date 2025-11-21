import yaml
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
import time


def yaml_to_occupancy_grid(yaml_file): # convert map image (yaml file) to 2D occupancy grid

    with open(yaml_file, 'r') as f: # open yaml file
        map_metadata = yaml.safe_load(f) # load data into dictionary 
    
    # extract map path & occupied + free threshold values from yaml file
    image_path = map_metadata["image"]
    occ_thresh = map_metadata["occupied_thresh"]
    free_thresh = map_metadata["free_thresh"]

    # open map image 
    img = Image.open(image_path).convert('L')  # convert map image to greyscale ('L')
    img = np.array(img) # convert greyscale image into array for later calculations

    normalized = img / 255.0 # set pixel value range

    grid = np.zeros_like(normalized, dtype=int) # create a grid filled with 0s (same size as map image)


    grid[normalized < occ_thresh] = 1  # set cells with values less than the occupied threshold values as obstacles = obstacles have values of 1
    grid[normalized > free_thresh] = 0  # set cells with values greater than the free threshold values as free = free have values of 1
    mask_unknown = (normalized <= occ_thresh) & (normalized >= free_thresh)  # create reprentation for cells with pixel values that are in between the occupied & free threshold values
    grid[mask_unknown] = -1 # set cells with values in between the occupied & free threshold values as unknown = unknown have values of -1                

    return grid # return 2D occupancy grid


def is_valid_node(node:tuple,grid): # determine if node is inside grid

    (row,col) = node # define node as variable with row & column values
    
    total_rows = 38 # set number of rows in grid
    total_cols = 30 # set number of columns in grid

    if row >= 0 and row < total_rows: # check if row index is inside grid
        if col >= 0 and col < total_cols: # check if column index is inside grid
            if grid[row][col] == 0: # check if grid cell is free & inside grid
                return True # grid cell is free & inside grid = can be moved to
    return False # grid cell is blocked with obstacle or outside grid = cannot be moved to


def neighbors(node:tuple): # determine the neighboring grid cells of node

    (row,col) = node # define node as variable with row & column values

    top_node = (row-1,col) # calculate grid cell above node
    bottom_node = (row+1,col) # calculate grid cell below node
    right_node = (row,col+1) # calculate grid cell to right of node
    left_node = (row,col-1) # calculate grid cell to left of node

    return [top_node,bottom_node,left_node,right_node] # return all values


def heuristic(node1,node2): # determine manhattan distance (chosen heuristic)

    (x1,y1) = node1 # set coordinates of node 1
    (x2,y2) = node2 # set coordinates of node 2
    manhattan_distance = abs(x1-x2)+abs(y1-y2) # calculate manhattan distance
    return manhattan_distance # return manhattan distance



def plot_grid(grid, start_node, goal_node, path): # visualize grid, start + goal nodes, & final path

    rows, cols = grid.shape # set rows = height & cols = width of grid

    fig, ax = plt.subplots(figsize=(10, 10)) # create 10x10 grid figure

    ax.imshow(grid, cmap="gray_r", origin="lower") # display grid with features ("gray_r" = white for free nodes & black for nodes with obstacles + "lower" = place (0,0) in bottom left)

    # format grid 
    ax.set_xticks(np.arange(-0.5, cols, 1))
    ax.set_yticks(np.arange(-0.5, rows, 1))

    ax.grid(which="both", color="lightgray", linewidth=0.5)

    # set x (columns) & y (rows) coordinates for plotting
    x_val = [c for (r, c) in path]
    y_val = [r for (r, c) in path]

    # plot start + goal nodes
    ax.scatter(start_node[1], start_node[0], marker="o", color="blue", label="start")
    ax.scatter(goal_node[1], goal_node[0], marker="o", color="yellow", label="goal")

    # plot path
    ax.plot(x_val, y_val, color="green", linewidth=2, label="path")

    # format data
    ax.set_title(" A* Algorithm)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()

    # display plotted data
    plt.show()

def inflate_grid(grid, inflation_cells): # inflate obstacles to ensure that the robot is always a safe enough distance away from them = prevents possible damage 
   
    rows, cols = grid.shape # set rows = height & cols = width of grid
    inflated = grid.copy() # make copy of grid to modify without overwriting original grid

    obstacle_indices = np.argwhere(grid == 1) # determine all obstacles in path (values that = 1 are obstacles / values that = 0 are free)

    for (r, c) in obstacle_indices: # for every obstacle cell with r = rows & c = columns
        # create inflated square around obstacle cell
        for dr in range(-inflation_cells, inflation_cells + 1): # dr = change in row = x
            for dc in range(-inflation_cells, inflation_cells + 1): # dc = change in column = y

                if dr*dr + dc*dc <= inflation_cells*inflation_cells: # use equation of circle to determine if (dr,dc) is inside radius of circle = used to make rounded inflated obstacles rather than square
                    # calculate coordinates of inflated cell
                    rr = r + dr
                    cc = c + dc

                    if 0 <= rr < rows and 0 <= cc < cols: # for cell within grid boundaries
                        inflated[rr, cc] = 1 # set as inflated obstacle
    return inflated # return grid with inflated obstacles
    

if __name__ == '__main__':
    
    grid = yaml_to_occupancy_grid("my_robot_map_real.yaml") # convert map image from yaml file to 2D occupancy grid
    resolution = 0.05 # resolution of grid in centimeters
    robot_radius = 0.105 # radius of robot in meters
    inflation_cells = int(robot_radius // resolution) -1 # calculate inflation based on resolution & radius               

    inflated_grid = inflate_grid(grid, inflation_cells) # create new grid with inflated objects
    
    # set goal & start node coordinates
    goal_node = (25, 17)
    start_node = (9,17)
    
    start_time = time.time() # record start time of algorithm
    total_exapnded_nodes , path, cost = BFS(start_node, goal_node, inflated_grid) # return total expanded nodes, path, & cost of path


    end_time = time.time() # record end time of algorithm
    total_time = (end_time - start_time) *100 # calculate total execution time of algorithm
    
    # display data
    print("shortest path:",path)
    print(f"run time: {total_time} ms") 
    print("path cost:", len(path))
    print("total expanded nodes", total_exapnded_nodes)

    # plot grid
    plot_grid(inflated_grid,start_node,goal_node,path)