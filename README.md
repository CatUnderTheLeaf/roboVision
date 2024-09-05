# roboVision project

A personalized voice-controlled robot-assistant. The base use-case scenarios:
1. _Search_: user asks robot to find object -> robot navigates around to find this object -> robot reports back search status and information about the found object
2. _Delivery_: user asks robot to bring the object -> robot performs _Search_ to find an object -> if the search is successful grabs the object and delivers it to the user
3. _Sort_: user asks robot to sort objects into different places by some criteria -> robor performs _Search_ to find sorting places -> robot performs _Delivery_ to sort all objects

I can build a LEGO Mindstorm robot (because I already have this one) with:
- a differential drive using two large motors for wheels and small rotating wheel for stability
- a smartphone used as a web-cam
- a gripper controlled by one medium motor
- a radar to get approximation of objects pose
- a bumper which presses a button if the collision is very near
- a possibility to use a color sensor
  
[Detailed plan and work-in-progress](https://github.com/users/CatUnderTheLeaf/projects/2)



---

> For later reference, maybe I will need some commands

A template project integrating ROS 2 and Gazebo simulator.

## Included packages

* `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_example_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_example_application` - holds ros2 specific code and configurations.

* `ros_gz_example_bringup` - holds launch files and high level utilities.


## Install

For using the template with Gazebo Fortress switch to the `fortress` branch of this repository, otherwise use the default branch `main` for Gazebo Harmonic onwards.

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation
   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

    ```bash
    export GZ_VERSION=fortress
    ```

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

### Use as template
Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

   ```bash
   mkdir -p ~/template_ws/src
   cd ~/template_ws/src
   wget https://raw.githubusercontent.com/gazebosim/ros_gz_project_template/main/template_workspace.yaml
   vcs import < template_workspace.yaml
   ```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/<ROS_DISTRO>/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_example_bringup diff_drive.launch.py
    ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
