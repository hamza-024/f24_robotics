# Homework 1 ROS2 Notes

## Setup and Launch Instructions

### Initial Setup

1. **Set up ROS environment variables**:
   Ensure ROS is configured properly on your system. Source the ROS setup file:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

   - **For Mac users**:
     ```bash
     export WEBOTS_HOME=/Applications/Webots.app
     python3 local_simulation_server.py
     ```

   - **For Windows users**:
     ```bash
     export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
     ```

2. **Clone the repository**:
   Fork and clone your own version of the repository:
   ```bash
   git clone <your github url for this repository>
   ```

3. **Build the package**:
   Navigate to the package directory and build the project:
   ```bash
   cd f24_robotics
   colcon build
   ```

4. **Source the installation**:
   Set up your environment to use the newly built package:
   ```bash
   source install/setup.bash
   ```

### Launching the World Files

The simulation initially launches `maze1_smallest.wbt`, renamed to `f23_robotics_1.wbt`. To switch to another world file, follow these steps:

- **Rename `maze.wbt` to `f23_robotics_1.wbt`**:
  Ensure you rename the desired world file to `f23_robotics_1.wbt` to replace the initial world file.

### Running the Wall-Follow Controller

To run the wall-follow controller, execute the following commands:

1. **Open a terminal and set up the ROS environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Navigate to your git repository**:
   Replace `<git-repo>` with the actual path to your cloned repository.
   ```bash
   cd <git-repo>
   ```

3. **Source the installation again**:
   ```bash
   source install/setup.bash
   ```

4. **Run the wall-follow controller**:
   ```bash
   ros2 run webots_ros2_homework1_python webots_ros2
   ```
