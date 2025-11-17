
from queue import PriorityQueue


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


def dijkstra(start_node,goal_node,grid): # determine the shortest path on grid


        open_list = PriorityQueue() # sort & store nodes based on smallest cost
        open_list.put((0,start_node)) # set 0 as cost of start node

        closed_list = set() # store nodes that have already been explored to prevent infinite loop

        parents = {start_node:None} # stores the parent of each visited node

        cost_from_start_node = {start_node:0} # stores the optimal cost 

        while not open_list.empty(): # run algorithm until goal is reached

            (current_cost,current_node) = open_list.get() # determine the smallest cost at current point

            if current_node == goal_node: # stop program if goal is already reached
                break
            if current_node in closed_list: # skip node if already explored before
                continue 
            else: # set the node as explored 
                closed_list.add(current_node)

            for next_node in neighbors(current_node): # explore neighbors
                if is_valid_node(next_node,grid) and next_node not in closed_list: # check if neighbor is free & insde grid
                    new_cost = current_cost+1 # increase cost
                    if next_node not in cost_from_start_node or new_cost < cost_from_start_node[next_node]: # check if neighbor has been explored or if cheaper cost path exists 
                        cost_from_start_node[next_node] = new_cost # update neighbor's cost 
                        parents[next_node] = current_node # update current node
                        open_list.put((new_cost,next_node)) # add neighbor to priority queue 


        path = [] # list to store shortest path found
        current_node = goal_node # set current node to goal node so that path is found by backtracking (begin at goal node & travel backwards to end at start node)
        while current_node != start_node: # run path search (via backtracking) until start node is reached
            path.append(current_node) # add every node to the path
            current_node = parents.get(current_node) # use parent of current node to move backwards along path
        path.append(start_node) # add the start node to the path
        path.reverse() # reverse the path to undo the backtracking (begin at start node & travel forwards to end at goal node)


    

    