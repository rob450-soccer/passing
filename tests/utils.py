"""
Utility functions including logging, process management, and math operations.
"""

import datetime
import logging
import os
import random
import signal
import subprocess
import sys
import threading
import time
import math


class Utils:
    def __init__(self):
        self.TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
        self.PASSING_DIR = os.path.dirname(self.TESTS_DIR)
        self.BASE_DIR = os.path.dirname(self.PASSING_DIR)
        self.RCSSSMJ_DIR = os.path.join(self.BASE_DIR, "rcssservermj")
        self.OBSTACLES_DIR = os.path.join(self.BASE_DIR, "obstacles")

        self.ROBOT_RADIUS = 0.23 # meters
        self.MIN_SAFE_DIST = 2 * self.ROBOT_RADIUS
        self.TIMEOUT_SECONDS = 60
        self.CLEANUP_TIMEOUT_SECONDS = 5


    ################################################################################
    ############################### SIMULATION UTILS ###############################
    ################################################################################


    def start_server(self, logger):
        server_process, _server_log_thread = self.popen_with_logged_output(
            [
                "hatch",
                "run",
                "rcssservermj",
                # "--no-render",
                # "--rerun",
                # "record",
                # "--rerunfile",
                # f"test1_{trial_count}",
            ],
            cwd=self.RCSSSMJ_DIR,
            logger=logger,
            label="server",
            start_new_session=True,
        )
        time.sleep(3)  # Give server time to initialize
        return server_process


    def start_player(self, player_id, start_x, start_y):
        player_process = subprocess.Popen(
            [
                "hatch",
                "run",
                "python3",
                "run_player.py",
                "--host",
                "localhost",
                "--port",
                "60000",
                "-n",
                str(player_id),
                "-t",
                "Team",
                "--spawn-x",
                str(start_x),
                "--spawn-y",
                str(start_y),
                "--spawn-rot",
                "0",
                "--verbose",
            ],
            cwd=self.PASSING_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        time.sleep(3)
        return player_process


    def start_obstacle(self, obstacle_id, obs_x, obs_y, logger):
        obstacle_process, _obs_log_thread = self.popen_with_logged_output(
            [
                "python3",
                "run_obstacles.py",
                "--number",
                str(obstacle_id),
                "--x",
                str(obs_x),
                "--y",
                str(obs_y),
            ],
            cwd=self.OBSTACLES_DIR,
            logger=logger,
            label="obstacle",
            start_new_session=True,
        )
        return obstacle_process


    def _monitor_kickoff_left(self, logger):
        kickoff = subprocess.run(
            ["python3", "monitor_client.py", "kickOff", "Left"],
            cwd=self.RCSSSMJ_DIR,
            timeout=2,
            capture_output=True,
            text=True,
        )
        for line in (kickoff.stdout or "").splitlines():
            logger.info(f"[monitor] {line}")
        for line in (kickoff.stderr or "").splitlines():
            logger.info(f"[monitor] {line}")

    def kickoff_only(self, logger):
        """Issue kickOff Left without repositioning the ball (simulation default spawn)."""
        try:
            self._monitor_kickoff_left(logger)
        except subprocess.TimeoutExpired as e:
            logger.debug("monitor_client.py timed out.")
            if e.stdout:
                for line in e.stdout.splitlines():
                    logger.info(f"[monitor] {line}")
            if e.stderr:
                for line in e.stderr.splitlines():
                    logger.info(f"[monitor] {line}")

    def place_ball_and_kickoff(self, ball_x, ball_y, logger):
        try:
            subprocess.run(
                ["python3", "monitor_client.py", "ball", f'"(pos {ball_x} {ball_y} 0)"'],
                cwd=self.RCSSSMJ_DIR,
                timeout=2,
                capture_output=True,
                text=True,
            )
            time.sleep(1)
            self._monitor_kickoff_left(logger)
        except subprocess.TimeoutExpired as e:
            logger.debug("monitor_client.py timed out.")
            if e.stdout:
                for line in e.stdout.splitlines():
                    logger.info(f"[monitor] {line}")
            if e.stderr:
                for line in e.stderr.splitlines():
                    logger.info(f"[monitor] {line}")


    ################################################################################
    ################################ LOGGING UTILS #################################
    ################################################################################


    def color(self, text: str, color: str) -> str:
        """
        Returns the given text wrapped in ANSI color codes for red, green, or blue.
        If an unsupported color is passed, it returns the original text.
        """
        colors = {"red": "\033[31m", "green": "\033[32m", "blue": "\033[34m"}

        reset = "\033[0m"
        color_code = colors.get(color.lower())

        if color_code:
            return f"{color_code}{text}{reset}"

        return text


    def setup_test_logging(self, test_name: str, debug=True) -> logging.Logger:
        """
        Configures the logger to output to both the console and a timestamped file.
        """
        os.makedirs("logs", exist_ok=True)

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_file_path = os.path.join("logs", f"{test_name}_{timestamp}.log")

        logger = logging.getLogger(test_name)
        logger.setLevel(logging.DEBUG)

        if not logger.handlers:
            # 1. File Handler: Saves everything to the log file
            file_handler = logging.FileHandler(log_file_path, mode="a", encoding="utf-8")
            file_handler.setLevel(logging.DEBUG)

            # 2. Stream Handler: Prints to the terminal
            # Routing to sys.stdout ensures your master script still captures it
            stream_handler = logging.StreamHandler(sys.stdout)
            stream_handler.setLevel(logging.DEBUG if debug else logging.INFO)

            formatter = logging.Formatter(
                "%(asctime)s:%(levelname)s: %(message)s", datefmt="%H:%M:%S"
            )
            file_handler.setFormatter(formatter)
            stream_handler.setFormatter(formatter)

            logger.addHandler(file_handler)
            logger.addHandler(stream_handler)

        return logger


    ################################################################################
    ################################ PROCESS UTILS #################################
    ################################################################################


    def popen_with_logged_output(self, cmd, cwd, logger, label, **popen_kwargs):
        """
        Start a subprocess with stdout/stderr merged, and log every line via logger.

        A background thread drains the pipe so verbose children do not fill the OS
        pipe buffer and block. Output is tagged with [label] in the log file.
        """
        proc = subprocess.Popen(
            cmd,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            **popen_kwargs,
        )

        def _drain():
            try:
                for line in iter(proc.stdout.readline, ""):
                    logger.info(f"[{label}] {line.rstrip()}")
            finally:
                if proc.stdout:
                    proc.stdout.close()

        thread = threading.Thread(target=_drain, name=f"log-{label}", daemon=True)
        thread.start()
        return proc, thread


    def terminate_process_tree(self, proc, logger, name):
        """Terminate a process group rooted at proc, then force kill if needed."""
        if not proc:
            return

        if proc.poll() is not None:
            return

        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return

        logger.info(f"cleanup {name} (pid={proc.pid})")
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return

        deadline = time.time() + 5
        while time.time() < deadline:
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        else:
            logger.warning(
                f"{name} process group did not terminate in time; sending SIGKILL."
            )
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        # Reap our direct child to avoid zombies in parent process.
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            logger.warning(f"{name} process still not reaped after kill attempts.")


    ################################################################################
    ################################## MATH UTILS ##################################
    ################################################################################


    def generate_random_point(self, min_x, max_x, min_y, max_y):
        """Generates a random float coordinate within the specified bounds."""
        return (
            round(random.uniform(min_x, max_x), 2),
            round(random.uniform(min_y, max_y), 2),
        )

    def point_to_segment_distance(self, p, a, b):
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

    def check_intersection(self, path_in_meters, obstacles):
        """Returns True if any segment of the path intersects with an obstacle."""
        if len(path_in_meters) < 2:
            return False
            
        for i in range(len(path_in_meters) - 1):
            segment_start = path_in_meters[i]
            segment_end = path_in_meters[i+1]
            
            for obs in obstacles:
                dist = self.point_to_segment_distance(obs, segment_start, segment_end)
                if dist < self.MIN_SAFE_DIST:
                    return True # Collision detected
        return False




    def move_to_google_drive(self, source, destination, logger):
        if subprocess.run(f"rclone copy {source} google_drive:/{destination} --progress", shell=True).returncode != 0:
            logger.error("Failed to upload files to Google Drive")
        else:
            logger.info(f"Uploaded files to Google Drive at {destination}/")
            subprocess.run(f"rm -f {source}", shell=True)  
            logger.info(f"Deleted files from {source}")