1. Install Ubuntu 20.04 LTS on your machine.
2. Install ROS Noetic Ninjemys on your machine.
```bash
sudo apt update
sudo apt upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools python-is-python3
sudo rosdep init
rosdep update
```
Resource : [ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)
3. Clone this repository.
```bash
git clone git@github.com:Edekheh/RoverAutonomy.git
```
4. Install the required packages.
```bash
cd ~/RoverAutonomy
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
```
5. Install additional packages.
```bash
sudo apt install ros-noetic-leo*
sudo apt install ros-noetic-grid-map*
```
6. Configure the workspace.
- Install vscode.
```bash
sudo snap install --classic code
```
- Install vscode extensions :
    - C/C++
    - CMake
    - Python
    - ROS
    - clangd
    - Git
    - GitHub Copilot
- Build the workspace.
```bash
cd ~/RoverAutonomy
./src/leo_autonomy/scripts/build.sh
```
7. Run the simulation.
If this is the first time you run the simulation, you need to source the workspace. You can add the following line to your ~/.bashrc file.
```bash
echo "source ~/RoverAutonomy/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Run the simulation.
```bash
roslaunch leo_autonomy sim.launch
```
8. Control the rover.
Use keys in the terminal where you run the simulation to control the rover.


