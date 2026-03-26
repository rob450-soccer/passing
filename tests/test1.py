import os
import sys
import time
import math
import random
import subprocess
import ast
import utils


TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PASSING_DIR = os.path.dirname(TESTS_DIR)
BASE_DIR = os.path.dirname(PASSING_DIR)
RCSSSMJ_DIR = os.path.join(BASE_DIR, "rcssservermj")
OBSTACLES_DIR = os.path.join(BASE_DIR, "obstacles")

ROBOT_RADIUS = 0.23 # meters
MIN_SAFE_DIST = 2 * ROBOT_RADIUS
TIMEOUT_SECONDS = 60

def generate_random_point(min_x, max_x, min_y, max_y):
    """Generates a random float coordinate within the specified bounds."""
    return (random.uniform(min_x, max_x), random.uniform(min_y, max_y))

def point_to_segment_distance(p, a, b):
    """Calculates the shortest distance from point p to the line segment ab."""
    px, py = p
    ax, ay = a
    bx, by = b

    l2 = (bx - ax)**2 + (by - ay)**2
    if l2 == 0:
        return math.hypot(px - ax, py - ay)

    # Find projection of point p onto the line extending the segment
    t = max(0, min(1, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / l2))
    
    proj_x = ax + t * (bx - ax)
    proj_y = ay + t * (by - ay)
    
    return math.hypot(px - proj_x, py - proj_y)

def check_intersection(path_in_meters, obstacles):
    """Returns True if any segment of the path intersects with an obstacle."""
    if len(path_in_meters) < 2:
        return False
        
    for i in range(len(path_in_meters) - 1):
        segment_start = path_in_meters[i]
        segment_end = path_in_meters[i+1]
        
        for obs in obstacles:
            dist = point_to_segment_distance(obs, segment_start, segment_end)
            if dist < MIN_SAFE_DIST:
                return True # Collision detected
    return False

def parse_path_from_log(log_line):
    """Extracts coordinates from the standard Python list format."""
    try:
        list_str = log_line.split("path: ")[1].strip()
        path_list = ast.literal_eval(list_str)
        return [(float(coord[0]), float(coord[1])) for coord in path_list]
    except (IndexError, ValueError, SyntaxError) as e:
        return []

def run_test():
    suite_start_time = time.time()
    logger = utils.setup_test_logging("test1")
    logger.info("Test 1")

    FIELD_MIN_X, FIELD_MAX_X = 0.5, 6.5
    FIELD_MIN_Y, FIELD_MAX_Y = -4.0, 4.0

    obstacle_configs = [
        [generate_random_point(FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y) for _ in range(3)] 
        for _ in range(10)
    ]
    start_positions = [
        generate_random_point(FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y) 
        for _ in range(10)
    ]

    total_trials = 100
    passed_trials = 0
    trial_count = 0

    player_id = 1
    obstacle_id = 1

    try:
        for obs_config in obstacle_configs:
            for start_x, start_y in start_positions:
                trial_count += 1
                logger.info(f"----- Running trial {trial_count}/{total_trials} -----")
                
                server_process = None
                processes = []

                try:
                    server_process = subprocess.Popen(
                        ["hatch", "run", "rcssservermj", "--no-render", "--rerun", "record"],
                        cwd=RCSSSMJ_DIR
                    )
                    time.sleep(3) # Give server time to initialize
                    
                    # Spawn obstacles
                    for obs_x, obs_y in obs_config:
                        p_obs = subprocess.Popen(
                            ["python3", "run_obstacles.py", "--number", str(obstacle_id), "--x", str(obs_x), "--y", str(obs_y)],
                            cwd=OBSTACLES_DIR
                        )
                        processes.append(p_obs)
                        obstacle_id += 1

                    # Spawn player
                    p_player = subprocess.Popen(
                        [
                            "hatch", "run", "python3", "-u", "run_player.py", 
                            "--host", "localhost", 
                            "--port", "60000", 
                            "-n", str(player_id), 
                            "-t", "Team1", 
                            "--spawn-x", str(start_x), 
                            "--spawn-y", str(start_y),
                            "--spawn-rot", "0"
                        ],
                        cwd=PASSING_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
                    )
                    processes.append(p_player)
                    player_id += 1
                    
                    time.sleep(1) # Give robots time to spawn before kickoff

                    # kickoff (with timeout in case it doesn't exit automatically)
                    try:
                        subprocess.run(["python3", "monitor_client.py", "kickOff", "Left"], cwd=RCSSSMJ_DIR, timeout=5)
                    except subprocess.TimeoutExpired:
                        logger.debug("monitor_client.py timed out, assuming kickoff signal was sent.")

                    # watch player output
                    raw_grid_path = []
                    grid_scale = None
                    reached_ball = False
                    start_time = time.time()

                    while True:
                        line = p_player.stdout.readline()
                        
                        if not line:
                            if p_player.poll() is not None:
                                break
                            continue
                        
                        # Log the run_player output to both the console and the log file
                        # We strip the newline character because the logger adds its own
                        logger.debug(line.rstrip('\n'))
                        
                        # Capture the scale (e.g., "grid world created with scale 10")
                        if "grid world created with scale" in line:
                            try:
                                grid_scale = float(line.split("scale ")[1].strip())
                            except (IndexError, ValueError):
                                logger.error(f"Failed to parse scale from line: {line.strip()}")

                        # Capture the path
                        if "robot_to_ball path:" in line:
                            raw_grid_path = parse_path_from_log(line)
                            
                        # Capture completion
                        if "robot reached ball" in line:
                            reached_ball = True
                            break
                            
                        # Timeout guard
                        if time.time() - start_time > TIMEOUT_SECONDS:
                            logger.error(utils.color(f"[FAIL] Trial {trial_count}: Timed out waiting for robot.", "red"))
                            break

                    # Evaluate Trial
                    if reached_ball:
                        if not raw_grid_path:
                            logger.error(utils.color(f"[FAIL] Trial {trial_count}: Reached ball, but no path was logged.", "red"))
                        elif grid_scale is None:
                            logger.error(utils.color(f"[FAIL] Trial {trial_count}: Grid scale was never logged.", "red"))
                        else:
                            path_in_meters = [
                                (gx / grid_scale, gy / grid_scale) 
                                for gx, gy in raw_grid_path
                            ]
                            
                            if check_intersection(path_in_meters, obs_config):
                                logger.error(utils.color(f"[FAIL] Trial {trial_count}: Path intersected with an obstacle.", "red"))
                            else:
                                passed_trials += 1
                                logger.info(utils.color(f"[PASS] Trial {trial_count}: Safe path executed.", "green"))
                    elif p_player.poll() is not None:
                        logger.error(utils.color(f"[FAIL] Trial {trial_count}: Player process crashed.", "red"))

                finally:
                    # Cleanup all processes before next trial
                    for p in processes:
                        p.terminate()
                        p.wait()
                    if server_process:
                        server_process.terminate()
                        server_process.wait()
    except KeyboardInterrupt:
        logger.error(utils.color("\nTest interrupted by user (Ctrl-C). Cleaned up background processes.", "red"))
        sys.exit(1)

    # Calculate elapsed time
    suite_end_time = time.time()
    logger.info(utils.color(f"\nTotal time elapsed: {suite_end_time - suite_start_time} seconds", "blue"))

    # Final Evaluation
    if passed_trials == total_trials:
        logger.info(utils.color(f"SUCCESS: {passed_trials}/{total_trials} trials passed.", "green"))
        sys.exit(0)
    else:
        logger.error(utils.color(f"FAILURE: {passed_trials}/{total_trials} trials passed.", "red"))
        sys.exit(1)

if __name__ == "__main__":
    run_test()