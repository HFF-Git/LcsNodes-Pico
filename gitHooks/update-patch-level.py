#-------------------------------------------------------------------------------------------------------------
# Simple program to update the GIT branch and patch level in the VCPU32 Version.h file. Every time we
# commit to Git, the patch level is increased. Also. we retrive the current GIt branch and update the
# GIT branch name. Both data items are displayed when teh simulator starts.
#
#-------------------------------------------------------------------------------------------------------------
import re
import sys
import subprocess

# Function to get the current Git branch
def get_git_branch():
    try:
        # Run 'git branch --show-current' to get the current branch name
        branch = subprocess.check_output(['git', 'branch', '--show-current'], stderr=subprocess.DEVNULL)
        return branch.decode('utf-8').strip()
    except subprocess.CalledProcessError:
        return None

# Function to increment the PATCH_LEVEL number
def increment_patch_level(line):
    match = re.search(r'const\s+int\s+SIM_PATCH_LEVEL\s*=\s*(\d+);', line)
    if match:
        current_value = int(match.group(1))
        incremented_value = current_value + 1
        updated_line = line.replace(str(current_value), str(incremented_value))
        return updated_line
    return line

# Function to update the GIT branch

def update_git_branch(line, branch_name):
    match = re.search(r'const\s+char\s+SIM_GIT_BRANCH\s*\[\s*]\s*=\s*".*?";', line)
    if match:
        updated_line = f'const char SIM_GIT_BRANCH[ ] = "{branch_name}";\n'
        return updated_line
    return line

# Function to update the file
def update_file(file_path):
    try:
        branch_name = get_git_branch() or "Unknown"
        
        with open(file_path, 'r') as file:
            lines = file.readlines()
        
        with open(file_path, 'w') as file:
            for line in lines:
                line = increment_patch_level(line)
                line = update_git_branch(line, branch_name)
                file.write(line)
        
        print(f"File '{file_path}' has been updated successfully.")
    
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
    except Exception as e:
        print(f"Error: {e}")

# Main function
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python increment_patch_level.py <file_path>")
    else:
        file_path = sys.argv[1]
        update_file(file_path)
