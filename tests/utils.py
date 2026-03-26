import logging
import os
import datetime
import sys


def color(text: str, color: str) -> str:
    """
    Returns the given text wrapped in ANSI color codes for red, green, or blue.
    If an unsupported color is passed, it returns the original text.
    """
    colors = {
        "red": "\033[31m",
        "green": "\033[32m",
        "blue": "\033[34m"
    }
    
    reset = "\033[0m"
    color_code = colors.get(color.lower())
    
    if color_code:
        return f"{color_code}{text}{reset}"
    
    return text


def setup_test_logging(test_name: str, debug=False) -> logging.Logger:
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
        file_handler = logging.FileHandler(log_file_path, mode='a', encoding='utf-8')
        file_handler.setLevel(logging.DEBUG)
        
        # 2. Stream Handler: Prints to the terminal 
        # Routing to sys.stdout ensures your master script still captures it
        stream_handler = logging.StreamHandler(sys.stdout)
        stream_handler.setLevel(logging.DEBUG if debug else logging.INFO)
        
        formatter = logging.Formatter('%(asctime)s:%(levelname)s: %(message)s', datefmt='%H:%M:%S')
        file_handler.setFormatter(formatter)
        stream_handler.setFormatter(formatter)
        
        logger.addHandler(file_handler)
        logger.addHandler(stream_handler)
        
    return logger
