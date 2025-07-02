import subprocess
import sys

def install_pip_packages(requirements_file="requirements.txt"):
    print(f"Installing Python packages from {requirements_file} ...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", requirements_file])
    print("Python packages installed successfully.\n")

def install_ros_packages(ros_packages_file="ros_packages.txt"):
    print(f"Installing ROS packages from {ros_packages_file} ...")
    with open(ros_packages_file, "r") as f:
        packages = [line.strip() for line in f if line.strip()]
    if packages:
        cmd = ["sudo", "apt", "install", "-y"] + packages
        subprocess.check_call(cmd)
        print("ROS packages installed successfully.\n")
    else:
        print("No ROS packages to install.\n")

if __name__ == "__main__":
    install_pip_packages()
    install_ros_packages()
