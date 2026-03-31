import glob
import subprocess
import sys
import os
import datetime


# Setup master logging directory and filename
start_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)
log_file_path = os.path.join(log_dir, f"all_tests_{start_time}.log")

# Helper function to print with color but log plain text
def log_and_print(console_msg, log_msg=None):
    if log_msg is None:
        log_msg = console_msg
    
    print(console_msg)
    with open(log_file_path, "a", encoding="utf-8") as log_file:
        log_file.write(log_msg + "\n")

# Find all files matching test*.py
test_files = sorted(glob.glob("test*.py"))

if not test_files:
    log_and_print("No test files found.")
    sys.exit(0)

passed = 0
total = len(test_files)

log_and_print("Starting tests...")
log_and_print("-" * 50)

for test_file in test_files:
    result = subprocess.run([sys.executable, test_file])
    
    # Determine pass/fail based strictly on the return code
    if result.returncode == 0:
        passed += 1
        log_and_print(f"\033[32m[PASS]\033[0m {test_file}", f"[PASS] {test_file}")
    else:
        log_and_print(f"\033[31m[FAIL]\033[0m {test_file}", f"[FAIL] {test_file}")
    
    log_and_print("-" * 50)

# Format final summary
file_summary = f"{passed} out of {total} tests passed."
if passed == total and total > 0:
    log_and_print(f"\033[32m{file_summary}\033[0m", file_summary)
else:
    log_and_print(f"\033[31m{file_summary}\033[0m", file_summary)

# Exit with an error code if any tests failed
if passed != total:
    sys.exit(1)
