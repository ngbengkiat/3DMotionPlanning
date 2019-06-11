from enum import Enum
from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
from bresenham import bresenham

def bres(x1, y1, x2, y2): 
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    cells = []
    
    # TODO: Determine valid grid cells
    m = (y2-y1)/(x2-x1)
    yn = y1
    xn = x1
    y = y1+m

    while xn < x2:
        cells.append([xn,yn])        
        if (y > yn+1):
            yn+=1
        else:
            xn+=1
            y += m
        #print(xn,yn)

    return np.array(cells)
    
def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, int(north_min), int(east_min), edges
    
def a_star(graph, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]

        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                wt = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + wt
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def heuristic(n1, n2):
    #TODO: define a heuristic
    
    return np.linalg.norm(np.array(n2) - np.array(n1))
    
def heuristic2(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)
    
def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.vstack((point(p1), point(p2), point(p3)))
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    if path is not None:
        #pruned_path = [p for p in path]
        # TODO: prune the path!
        pruned_path = []
        pruned_path.append(path[0])
        for i in range(1,len(path)-1):
            if not collinearity_check(pruned_path[-1], path[i], path[i+1]):
                pruned_path.append(path[i])

    else:
        pruned_path = path
    pruned_path.append(path[-1])
    return pruned_path

def find_start_goal(graph, start, goal):
    # TODO: find start and goal on skeleton
    # Some useful functions might be:
        # np.nonzero()
        # np.transpose()
        # np.linalg.norm()
        # np.argmin()
    #print(type(graph.nodes()), len(graph.nodes()))
    dist_start = 1000
    dist_goal = 1000
    goal_node = None
    start_node = None
    for n in graph.nodes():
        # Array of dist to start point
        tmp_s = np.linalg.norm(np.array(n)-np.array(start))
        tmp_g = np.linalg.norm(np.array(n)-np.array(goal))
        if tmp_s < dist_start:
            dist_start = tmp_s
            start_node = n
        if tmp_g < dist_goal:
            dist_goal = tmp_g
            goal_node = n
            
    

    return start_node, goal_node
    