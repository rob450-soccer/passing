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

################################################################################
################################ LOGGING UTILS #################################
################################################################################


def color(text: str, color: str) -> str:
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


def setup_test_logging(test_name: str, debug=True) -> logging.Logger:
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


def popen_with_logged_output(cmd, cwd, logger, label, **popen_kwargs):
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


def terminate_process_tree(proc, logger, name):
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
################################ MATH UTILS #################################
################################################################################


def generate_random_point(min_x, max_x, min_y, max_y):
    """Generates a random float coordinate within the specified bounds."""
    return (
        round(random.uniform(min_x, max_x), 2),
        round(random.uniform(min_y, max_y), 2),
    )
