import pygame
import sys
import threading
import time
import argparse
import traceback
from algorithms import ana_star, theta_star, ana_theta_star, a_star
from world import GridWorld

# --- Configuration & Constants ---
WIDTH, HEIGHT = 720, 720
ROWS, COLS = 40, 40
CELL_WIDTH = WIDTH // COLS
CELL_HEIGHT = HEIGHT // ROWS

# Colors
WHITE = (250, 250, 250)
BLACK = (40, 40, 40)
GRAY = (200, 200, 200)
GREEN = (46, 204, 113)
RED = (231, 76, 60)
BLUE = (52, 152, 219)

def main():
    parser = argparse.ArgumentParser(description="Pathfinding Simulation")
    parser.add_argument('--algo', type=str, choices=['ana', 'theta', 'ana_theta', 'a_star'], default='theta', help='Algorithm to use {theta, ana, ana_theta, a_star}, default theta')
    parser.add_argument('--goal', type=str, choices=['single', 'set'], default='single', help='Shape of the goal (single or set)')
    args = parser.parse_args()
    
    current_algo = args.algo
    current_goal_shape = args.goal
    
    if current_algo == 'ana':
        display_algo = "ANA*"
    elif current_algo == 'theta':
        display_algo = "Theta*"
    elif current_algo == 'a_star':
        display_algo = "A*"
    else:
        display_algo = "ANA* Theta*"

    pygame.init()
    pygame.font.init() # Initialize font module
    font = pygame.font.SysFont('Arial', 20, bold=True) # Create a font object

    win = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"{display_algo} Pathfinding Simulation (Draw Walls: Left Click | Erase: Right Click)")
    clock = pygame.time.Clock()

    # Track the raw user inputs (0 = Empty, 1 = Obstacle)
    raw_grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]
    grid_world = GridWorld(raw_grid)
    
    start_pos = (5, 5)
    end_pos = (ROWS - 6, COLS - 6)
    
    # State tracking
    current_path = []
    last_drawn_path = [] # Keeps track of the last path to count iterations
    improvements = 0     # Counter for anytime optimizations
    dragging_start = False
    dragging_end = False

    timing_info = {'total': 0.0, 'calls': 0}

    def get_goal_points(center_pos, shape):
        """Returns a list of grid coordinates or a single coordinate tuple that belong to the goal."""
        r, c = center_pos
        if shape == 'set':
            pts = [(r, c)]
            if r > 0: pts.append((r - 1, c))
            if r < ROWS - 1: pts.append((r + 1, c))
            return pts
        return (r, c)

    def planner_worker(world, start, end, path_list, timing, algo, goal_shape):
        """Wrapper to measure the actual execution time of the planner."""
        t0 = time.perf_counter()
        
        goal_pts = get_goal_points(end, goal_shape)
        final_path = None
        
        try:
            if algo == 'ana':
                final_path = ana_star(world, start, goal_pts, path_list)
            elif algo == 'theta':
                # Theta* doesn't accept the path_list argument to mutate, it just returns the final path
                final_path = theta_star(world, start, goal_pts)
            elif algo == 'a_star':
                final_path = a_star(world, start, goal_pts)
            elif algo == 'ana_theta':
                final_path = ana_theta_star(world, start, goal_pts, path_list)
        except Exception as e:
            print(f"\n\n[Algorithm Crash] '{algo}' threw a Python error:")
            traceback.print_exc()
            print()
            return # Safely exit the thread without locking up Pygame
            
        t1 = time.perf_counter()
        
        # Ensure Pygame gets the final path even if the list wasn't mutated in-place
        if final_path is not None:
            path_list[:] = final_path
        else:
            path_list[:] = [] # Clear the visual path if the algorithm returned None (failed to find path)
        
        timing['total'] += (t1 - t0)
        timing['calls'] += 1
        avg_ms = (timing['total'] / timing['calls']) * 1000
        
        # Format for terminal print
        print(f"\r{display_algo} - Average time to calculate a path: {avg_ms:.1f} ms", end="", flush=True)

    # Launch the initial calculation in a background thread
    threading.Thread(
        target=planner_worker, 
        args=(grid_world, start_pos, end_pos, current_path, timing_info, current_algo, current_goal_shape), 
        daemon=True
    ).start()

    while True:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                print() # newline after carriage returns
                sys.exit()

            # Check for dragging start/end points
            if event.type == pygame.MOUSEBUTTONDOWN:
                x, y = pygame.mouse.get_pos()
                row, col = y // CELL_HEIGHT, x // CELL_WIDTH
                
                if event.button == 1: # Left click
                    current_goals = get_goal_points(end_pos, current_goal_shape)
                    goal_list = current_goals if isinstance(current_goals, list) else [current_goals]
                    
                    if (row, col) == start_pos:
                        dragging_start = True
                    elif (row, col) in goal_list:
                        dragging_end = True

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging_start = False
                    dragging_end = False

        # 2. Continuous Input Handling (Drawing / Dragging)
        x, y = pygame.mouse.get_pos()
        row, col = min(max(y // CELL_HEIGHT, 0), ROWS - 1), min(max(x // CELL_WIDTH, 0), COLS - 1)
        grid_changed = False

        current_goals = get_goal_points(end_pos, current_goal_shape)
        goal_list = current_goals if isinstance(current_goals, list) else [current_goals]

        if dragging_start and grid_world.is_free([row, col]) and (row, col) not in goal_list:
            start_pos = (row, col)
            grid_changed = True
        elif dragging_end:
            # Ensure all points of the new goal shape are free of walls
            new_goal_pts = get_goal_points((row, col), current_goal_shape)
            new_goal_list = new_goal_pts if isinstance(new_goal_pts, list) else [new_goal_pts]
            valid_pos = True
            for pt in new_goal_list:
                if not grid_world.is_free([pt[0], pt[1]]) or pt == start_pos:
                    valid_pos = False
                    break
            
            if valid_pos and end_pos != (row, col):
                end_pos = (row, col)
                grid_changed = True
        else:
            # Drawing and erasing walls
            left_click, _, right_click = pygame.mouse.get_pressed()
            if left_click and not dragging_start and not dragging_end:
                if (row, col) != start_pos and (row, col) not in goal_list and raw_grid[row][col] != 1:
                    raw_grid[row][col] = 1
                    grid_changed = True
            elif right_click:
                if raw_grid[row][col] != 0:
                    raw_grid[row][col] = 0
                    grid_changed = True

        # Only recalculate if the grid/positions actually updated
        if grid_changed:
            grid_world.update_grid(raw_grid)
            
            # Re-initialize current_path as a fresh list so the new thread 
            # doesn't conflict with any leftover old threads finishing up.
            current_path = [] 
            last_drawn_path = []
            improvements = 0
            
            # Launch Planner as a background process
            threading.Thread(
                target=planner_worker, 
                args=(grid_world, start_pos, end_pos, current_path, timing_info, current_algo, current_goal_shape), 
                daemon=True
            ).start()

        # 3. Rendering
        win.fill(WHITE)

        # Draw Obstacles and Inflation Layer
        for r in range(ROWS):
            for c in range(COLS):
                val = grid_world._grid[r, c]
                if val >= 1:
                    pygame.draw.rect(win, BLACK, (c * CELL_WIDTH, r * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT))
                elif 0 < val < 1: 
                    pygame.draw.rect(win, GRAY, (c * CELL_WIDTH, r * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT))

        # Draw Grid Lines
        for r in range(ROWS):
            pygame.draw.line(win, GRAY, (0, r * CELL_HEIGHT), (WIDTH, r * CELL_HEIGHT))
        for c in range(COLS):
            pygame.draw.line(win, GRAY, (c * CELL_WIDTH, 0), (c * CELL_WIDTH, HEIGHT))

        # Draw Start and End Blocks
        pygame.draw.rect(win, GREEN, (start_pos[1] * CELL_WIDTH, start_pos[0] * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT))
        
        current_goals = get_goal_points(end_pos, current_goal_shape)
        goal_list = current_goals if isinstance(current_goals, list) else [current_goals]
        for gp in goal_list:
            pygame.draw.rect(win, RED, (gp[1] * CELL_WIDTH, gp[0] * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT))

        # Draw the Path
        if current_path:
            try:
                # Track improvements by seeing if the background thread gave us a new path
                if current_path != last_drawn_path:
                    # Only count it as an improvement if we already drew a worse path
                    if len(last_drawn_path) > 0: 
                        improvements += 1
                    last_drawn_path = current_path[:]

                # We copy the list using [:] to avoid crashes if the background thread 
                # changes the list size at the exact millisecond we try to draw it.
                path_to_draw = current_path[:] 
                pixel_points = [((c + 0.5) * CELL_WIDTH, (r + 0.5) * CELL_HEIGHT) for r, c in path_to_draw]
                if len(pixel_points) > 1:
                    pygame.draw.lines(win, BLUE, False, pixel_points, 5)
            except RuntimeError:
                pass # Fail gracefully for one frame if thread collision occurs

        # --- Draw On-Screen Metrics ---
        # Draw a semi-transparent background box for text readability
        # Both ANA* and ANA* Theta* are anytime algorithms, so show the taller overlay for both
        is_anytime = current_algo in ['ana', 'ana_theta']
        overlay_height = 70 if is_anytime else 40
        
        overlay = pygame.Surface((WIDTH, overlay_height))
        overlay.set_alpha(220)
        overlay.fill(WHITE)
        win.blit(overlay, (0, 0))

        # Render text surfaces
        algo_text = font.render(f"Algorithm: {display_algo}", True, BLUE)
        
        avg_time = ((timing_info['total'] / timing_info['calls']) * 1000) if timing_info['calls'] > 0 else 0
        time_text = font.render(f"Avg Run Time: {avg_time:.2f} ms", True, BLACK)
        
        # Blit text to the window, distributing it horizontally and vertically
        win.blit(algo_text, (20, 10))
        win.blit(time_text, (500, 10))
        
        # Only show path length and improvements for the anytime variants
        if is_anytime:
            path_text = font.render(f"Path Length: {len(last_drawn_path)}", True, BLACK)
            iter_text = font.render(f"Path Improvements: {improvements}", True, BLACK)
            win.blit(path_text, (20, 40))
            win.blit(iter_text, (250, 40))

        pygame.display.flip()
        clock.tick(60) # Limit to 60 FPS

if __name__ == "__main__":
    main()