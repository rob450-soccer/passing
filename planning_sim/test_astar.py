import heapq
import math

class Node:
    """A node class for Theta* Pathfinding"""
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = float('inf') # Cost from start to current node
        self.h = 0            # Heuristic cost to goal
        self.f = float('inf') # Total cost

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    """Calculates the Euclidean distance for true any-angle cost calculation."""
    return math.hypot(a[0] - b[0], a[1] - b[1])

def line_of_sight(grid, start, end):
    """
    Checks if there is a clear line of sight between two points 
    using Bresenham's Line Algorithm.
    Returns True if clear, False if an obstacle is hit.
    """
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        # Check bounds
        if not (0 <= x0 < len(grid) and 0 <= y0 < len(grid[0])):
            return False
            
        # Check for obstacle
        if grid[x0][y0] == 1:
            return False
            
        # Reached the target
        if x0 == x1 and y0 == y1:
            break
            
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
            
    return True

def thetastar(grid, start, end):
    """
    Executes the Theta* algorithm.
    Returns a list of tuples as an any-angle path from start to end.
    """
    open_list = []
    closed_set = set()
    nodes = {} # Keep track of Node objects by their position
    
    # Initialize start node
    start_node = Node(start)
    start_node.g = 0
    start_node.h = heuristic(start, end)
    start_node.f = start_node.h
    nodes[start] = start_node
    
    heapq.heappush(open_list, (start_node.f, start_node.position))
    
    # 8-way directional movement for grid exploration
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while open_list:
        current_f, current_pos = heapq.heappop(open_list)
        current_node = nodes[current_pos]
        
        if current_pos in closed_set:
            continue
            
        closed_set.add(current_pos)
        
        # Goal reached, backtrack to build the path
        if current_pos == end:
            path = []
            curr = current_node
            while curr is not None:
                path.append(curr.position)
                curr = curr.parent
            return path[::-1] # Return reversed path
            
        # Expand neighbors
        for dx, dy in directions:
            neighbor_pos = (current_pos[0] + dx, current_pos[1] + dy)
            
            # Check grid bounds
            if not (0 <= neighbor_pos[0] < len(grid) and 0 <= neighbor_pos[1] < len(grid[0])):
                continue
                
            # Check for obstacles
            if grid[neighbor_pos[0]][neighbor_pos[1]] == 1:
                continue
                
            # Prevent squeezing diagonally through two corner blocks
            if dx != 0 and dy != 0:
                if grid[current_pos[0] + dx][current_pos[1]] == 1 and grid[current_pos[0]][current_pos[1] + dy] == 1:
                    continue
                    
            if neighbor_pos in closed_set:
                continue
                
            # Create or retrieve neighbor node
            if neighbor_pos not in nodes:
                nodes[neighbor_pos] = Node(neighbor_pos)
            neighbor_node = nodes[neighbor_pos]
            
            # ==========================================
            # THETA* CORE LOGIC (Path 1 vs Path 2)
            # ==========================================
            if current_node.parent and line_of_sight(grid, current_node.parent.position, neighbor_pos):
                # Path 2: Line of sight exists from parent to neighbor. Bypass current!
                parent_pos = current_node.parent.position
                cost = current_node.parent.g + heuristic(parent_pos, neighbor_pos)
                
                if cost < neighbor_node.g:
                    neighbor_node.g = cost
                    neighbor_node.f = cost + heuristic(neighbor_pos, end)
                    neighbor_node.parent = current_node.parent
                    heapq.heappush(open_list, (neighbor_node.f, neighbor_pos))
            else:
                # Path 1: No line of sight. Behave like standard A*
                cost = current_node.g + heuristic(current_pos, neighbor_pos)
                
                if cost < neighbor_node.g:
                    neighbor_node.g = cost
                    neighbor_node.f = cost + heuristic(neighbor_pos, end)
                    neighbor_node.parent = current_node
                    heapq.heappush(open_list, (neighbor_node.f, neighbor_pos))
                    
    return None # No path found