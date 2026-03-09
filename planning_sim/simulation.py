import pygame
import sys
from test_astar import thetastar

# --- Configuration & Constants ---
WIDTH, HEIGHT = 800, 800
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
    pygame.init()
    win = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("A* Simulation (Draw Walls: Left Click | Erase: Right Click)")
    clock = pygame.time.Clock()

    # 0 = Empty, 1 = Obstacle
    grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]
    
    start_pos = (5, 5)
    end_pos = (ROWS - 6, COLS - 6)
    
    # State tracking
    path = thetastar(grid, start_pos, end_pos)
    dragging_start = False
    dragging_end = False

    while True:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # Check for dragging start/end points
            if event.type == pygame.MOUSEBUTTONDOWN:
                x, y = pygame.mouse.get_pos()
                row, col = y // CELL_HEIGHT, x // CELL_WIDTH
                
                if event.button == 1: # Left click
                    if (row, col) == start_pos:
                        dragging_start = True
                    elif (row, col) == end_pos:
                        dragging_end = True

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging_start = False
                    dragging_end = False

        # 2. Continuous Input Handling (Drawing / Dragging)
        x, y = pygame.mouse.get_pos()
        row, col = min(max(y // CELL_HEIGHT, 0), ROWS - 1), min(max(x // CELL_WIDTH, 0), COLS - 1)
        grid_changed = False

        if dragging_start and grid[row][col] == 0 and (row, col) != end_pos:
            start_pos = (row, col)
            grid_changed = True
        elif dragging_end and grid[row][col] == 0 and (row, col) != start_pos:
            end_pos = (row, col)
            grid_changed = True
        else:
            # Drawing and erasing walls
            left_click, _, right_click = pygame.mouse.get_pressed()
            if left_click and not dragging_start and not dragging_end:
                if (row, col) != start_pos and (row, col) != end_pos and grid[row][col] == 0:
                    grid[row][col] = 1
                    grid_changed = True
            elif right_click:
                if grid[row][col] == 1:
                    grid[row][col] = 0
                    grid_changed = True

        # Only recalculate the path if the grid/positions actually updated
        if grid_changed:
            path = thetastar(grid, start_pos, end_pos)

        # 3. Rendering
        win.fill(WHITE)

        # Draw Obstacles
        for r in range(ROWS):
            for c in range(COLS):
                if grid[r][c] == 1:
                    pygame.draw.rect(win, BLACK, (c * CELL_WIDTH, r * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT))

        # Draw Grid Lines
        for r in range(ROWS):
            pygame.draw.line(win, GRAY, (0, r * CELL_HEIGHT), (WIDTH, r * CELL_HEIGHT))
        for c in range(COLS):
            pygame.draw.line(win, GRAY, (c * CELL_WIDTH, 0), (c * CELL_WIDTH, HEIGHT))

        # Draw Start and End Blocks
        pygame.draw.rect(win, GREEN, (start_pos[1] * CELL_WIDTH, start_pos[0] * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT))
        pygame.draw.rect(win, RED, (end_pos[1] * CELL_WIDTH, end_pos[0] * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT))

        # Draw the Theta* any-angle Path
        if path:
            # Offset points to the center of the cells for cleaner lines
            pixel_points = [((c + 0.5) * CELL_WIDTH, (r + 0.5) * CELL_HEIGHT) for r, c in path]
            if len(pixel_points) > 1:
                pygame.draw.lines(win, BLUE, False, pixel_points, 5)

        pygame.display.flip()
        clock.tick(60) # Limit to 60 FPS

if __name__ == "__main__":
    main()