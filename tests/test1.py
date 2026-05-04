import ast
import datetime
import glob
import math
import os
import subprocess
import sys
import time

from utils import Utils


util = Utils()

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PASSING_DIR = os.path.dirname(TESTS_DIR)
BASE_DIR = os.path.dirname(PASSING_DIR)
RCSSSMJ_DIR = os.path.join(BASE_DIR, "rcssservermj")
OBSTACLES_DIR = os.path.join(BASE_DIR, "obstacles")

ROBOT_RADIUS = 0.23 # meters
MIN_SAFE_DIST = 2 * ROBOT_RADIUS
TIMEOUT_SECONDS = 60
CLEANUP_TIMEOUT_SECONDS = 5


def parse_path_from_log(log_line):
    """Extracts coordinates from the standard Python list format."""
    try:
        list_str = log_line.split("path: ")[1].strip()
        path_list = ast.literal_eval(list_str)
        return [(float(coord[0]), float(coord[1])) for coord in path_list]
    except (IndexError, ValueError, SyntaxError) as e:
        return []


def collect_player_paths(player_process, trial_count, logger):
    paths_read = {
        "robot_to_ball": False,
        "scoring": False,
        "passing": False,
        "robot_to_receive": False,
    }
    paths = []
    grid_scale = None
    start_time = time.time()

    while True:
        line = player_process.stdout.readline()

        if not line:
            if player_process.poll() is not None:
                break
            continue

        if all(paths_read.values()):
            break

        line = line.rstrip("\n")
        logger.info(f"[player] {line}")

        if "grid world created with scale" in line:
            try:
                grid_scale = float(line.split("scale ")[1].strip())
            except (IndexError, ValueError):
                logger.error(f"Failed to parse scale from line: {line.strip()}")

        if "robot_to_ball path:" in line:
            paths.append(parse_path_from_log(line))
            paths_read["robot_to_ball"] = True

        if "scoring path:" in line:
            paths.append(parse_path_from_log(line))
            paths_read["scoring"] = True

        if "passing path:" in line:
            paths.append(parse_path_from_log(line))
            paths_read["passing"] = True

        if "robot_to_receive path:" in line:
            paths.append(parse_path_from_log(line))
            paths_read["robot_to_receive"] = True

        if time.time() - start_time > TIMEOUT_SECONDS:
            logger.error(util.color(f"[FAIL] Trial {trial_count}: Timed out waiting for robot.", "red"))
            break

    return paths_read, paths, grid_scale


def evaluate_trial(paths_read, paths, grid_scale, obs_config, trial_count, player_process, logger):
    if all(paths_read.values()):
        if len(paths) < 1:
            logger.error(util.color(f"[FAIL] Trial {trial_count}: Paths were not logged.", "red"))
            return False
        if grid_scale is None:
            logger.error(util.color(f"[FAIL] Trial {trial_count}: Grid scale was never logged.", "red"))
            return False

        for path in paths:
            path_in_meters = [(gx / grid_scale, gy / grid_scale) for gx, gy in path]
            if util.check_intersection(path_in_meters, obs_config):
                logger.error(
                    util.color(f"[FAIL] Trial {trial_count}: A path intersected with an obstacle.", "red")
                )
                return False
        logger.info(util.color(f"[PASS] Trial {trial_count}: Safe paths planned.", "green"))
        return True

    if player_process.poll() is not None:
        logger.error(util.color(f"[FAIL] Trial {trial_count}: Player process crashed.", "red"))

    return False


def cleanup_trial_processes(player_processes, obstacle_processes, server_process, logger):
    logger.info(util.color("Cleaning up", "yellow"))
    for i, process in enumerate(player_processes, start=1):
        util.terminate_process_tree(process, logger, f"player_{i}")
    for i, process in enumerate(obstacle_processes, start=1):
        util.terminate_process_tree(process, logger, f"obstacle_{i}")
    util.terminate_process_tree(server_process, logger, "server")


def run_test():
    suite_start_time = time.time()
    logger = util.setup_test_logging("test1")
    logger.info("Test 1")

    FIELD_MIN_X, FIELD_MAX_X = 0.5, 6.5
    FIELD_MIN_Y, FIELD_MAX_Y = -4.0, 4.0

    obstacle_configs = [
        [util.generate_random_point(FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y) for _ in range(3)] 
        for _ in range(10)
    ]
    start_positions = [
        util.generate_random_point(FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y) 
        for _ in range(10)
    ]
    ball_positions = [
        util.generate_random_point(FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y) 
        for _ in range(10)
    ]

    total_trials = 100
    passed_trials = 0
    trial_count = 0

    exit_code = 0

    TEST_DRIVE_FOLDER = f"Test1_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    if subprocess.run(f"rclone mkdir google_drive:/rob450-data/Verification/{TEST_DRIVE_FOLDER}", shell=True).returncode != 0:
        logger.error(f"Failed to create Google Drive folder for data: rob450-data/Verification/{TEST_DRIVE_FOLDER}")
        sys.exit(1)
    logger.info(f"Created Google Drive folder for data: rob450-data/Verification/{TEST_DRIVE_FOLDER}")

    try:
        for obs_config in obstacle_configs:
            for i in range(len(start_positions)):
                trial_count += 1
                start_x, start_y = start_positions[i]
                ball_x, ball_y = ball_positions[i]
                logger.info(f"{'-'*50} Running trial {trial_count}/{total_trials} {'-'*50}")
                logger.info(f"Obstacle configuration: {obs_config}")
                logger.info(f"Start position: ({start_x}, {start_y})")
                logger.info(f"Ball position: ({ball_x}, {ball_y})")
                
                server_process = None
                player_processes = []
                obstacle_processes = []

                player_id = 1
                obstacle_id = 1

                try:
                    server_process = util.start_server(logger)

                    p_player = util.start_player(player_id, start_x, start_y)
                    player_processes.append(p_player)

                    for obs_x, obs_y in obs_config:
                        obstacle_process = util.start_obstacle(obstacle_id, obs_x, obs_y, logger)
                        obstacle_processes.append(obstacle_process)
                        obstacle_id += 1

                    time.sleep(1)  # Give robots time to spawn before kickoff
                    util.place_ball_and_kickoff(ball_x, ball_y, logger)
                    paths_read, paths, grid_scale = collect_player_paths(p_player, trial_count, logger)

                    if evaluate_trial(
                        paths_read,
                        paths,
                        grid_scale,
                        obs_config,
                        trial_count,
                        p_player,
                        logger,
                    ):
                        passed_trials += 1

                finally:
                    cleanup_trial_processes(player_processes, obstacle_processes, server_process, logger)

                    # upload recordings to Google Drive
                    # util.move_to_google_drive(f"{RCSSSMJ_DIR}/recordings/*.rrd", f"rob450-data/Verification/{TEST_DRIVE_FOLDER}", logger)
                        
    except KeyboardInterrupt:
        logger.error(util.color("Test interrupted by user (Ctrl-C). Cleaned up background processes.", "red"))
        exit_code = 1

    # Calculate elapsed time
    suite_end_time = time.time()
    logger.info(util.color(f"Total time elapsed: {(suite_end_time - suite_start_time):.2f} seconds", "blue"))

    # Final Evaluation
    if passed_trials == total_trials:
        logger.info(util.color(f"SUCCESS: {passed_trials}/{total_trials} trials passed.", "green"))
    else:
        logger.error(util.color(f"FAILURE: {passed_trials}/{total_trials} trials passed.", "red"))
        exit_code = 1

    # # upload logs to Google Drive (use --include so rclone applies the pattern; shell * is unreliable)
    # logs_dir = os.path.join(TESTS_DIR, "logs")
    # rclone_dest = f"google_drive:/rob450-data/Verification/{TEST_DRIVE_FOLDER}"
    # if subprocess.run(
    #     ["rclone", "copy", logs_dir, rclone_dest, "--include", "*.log", "--progress"],
    # ).returncode != 0:
    #     logger.error("Failed to upload logs to Google Drive")
    # else:
    #     logger.info(f"Uploaded logs to Google Drive at rob450-data/Verification/{TEST_DRIVE_FOLDER}/")
    #     for path in glob.glob(os.path.join(logs_dir, "*.log")):
    #         os.remove(path)
    #     logger.info(f"Deleted local logs from {logs_dir}")

    sys.exit(exit_code)

if __name__ == "__main__":
    run_test()
