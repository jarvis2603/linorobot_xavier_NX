# linorobot_xavier_NX
install linorobot ROS2 HUMBLE pkg in jetson xavier Nx

1.install pkg
    sudo apt install libboost-python-dev python3-dev build-essential

    sudo apt remove --purge librealsense2*

    sudo rm -f /etc/apt/sources.list.d/librealsense.list

    wget -qO - https://librealsense.intel.com/Debian/apt-repo/apt-repo-key.pub | sudo apt-key add -

    sudo apt-add-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

    sudo apt update

    sudo apt install librealsense2-utils librealsense2-dev

    sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

    sudo apt remove --purge cmake

    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -

    sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'

    sudo apt update

2. Host Machine / Development Computer - Gazebo Simulation (Optional)

This step is only required if you plan to use Gazebo later. This comes in handy if you want to fine-tune parameters (ie. SLAM Toolbox, AMCL, Nav2) or test your applications on a virtual robot.
2.1 Install linorobot2 Package

Install linorobot2 package on the host machine:

cd <host_machine_ws>
git clone -b $ROS_DISTRO https://github.com/linorobot/linorobot2 src/linorobot2
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
colcon build
source install/setup.bash

    microxrcedds_agent and micro_ros_agent dependency checks are skipped to prevent this issue of finding its keys. This means that you have to always add --skip-keys microxrcedds_agent --skip-keys micro_ros_agent whenever you have to run rosdep install on the ROS2 workspace where you installed linorobot2.

2.2 Define Robot Type

Set LINOROBOT2_BASE env variable to the type of robot base used. Available env variables are 2wd, 4wd, and mecanum. For example:

echo "export LINOROBOT2_BASE=2wd" >> ~/.bashrc
source ~/.bashrc

You can skip the next step (Host Machine - RVIZ Configurations) since this package already contains the same RVIZ configurations to visualize the robot.
3. Host Machine - RVIZ Configuration

Install linorobot2_viz package to visualize the robot remotely specifically when creating a map or initializing/sending goal poses to the robot. The package has been separated to minimize the installation required if you're not using the simulation tools on the host machine.

cd <host_machine_ws>
git clone https://github.com/linorobot/linorobot2_viz src/linorobot2_viz
rosdep update && rosdep install --from-path src --ignore-src -y 
colcon build
source install/setup.bash

4. Docker Configuration

Docker can be used to run linorobot2 on a host machine for simulation. This can be useful if you aren't running Ubuntu 24.04 with ROS-Jazzy installed.

If you don't already have docker installed, you can install it using the docker installation instructions for your operating system. Be sure to follow the post-install instructions.

You might need to customize the docker/.env file if for example, you want to change the default robot type. Build the docker image:

git clone https://github.com/linorobot/linorobot2.git
cd linorobot2/docker
docker compose build

If you get a "permission denied" error running docker, follow the steps in the docker post install instructions to add yourself to the docker group, so that you don't have to use sudo to run docker commands.
Hardware and Robot Firmware

All the hardware documentation and robot microcontroller's firmware can be found here.
URDF
1. Define robot properties

linorobot2_description package has parameterized xacro files that can help you kickstart writing the robot's URDF. Open <robot_type>.properties.urdf.xacro in linorobot2_description/urdf directory and change the values according to the robot's specification/dimensions. All pose definitions must be measured from the base_link (center of base) and wheel positions (ie wheel_pos_x) are referring to wheel 1.

For custom URDFs, you can change the urdf_path in description.launch.py found in linorobot2_description/launch directory.

Robot Orientation:

--------------FRONT--------------

WHEEL1 WHEEL2 (2WD/4WD)

WHEEL3 WHEEL4 (4WD)

--------------BACK--------------

Build the robot computer's workspace to load the new URDF:

cd <robot_computer_ws>
colcon build

The same changes must be made on the host machine's <robot_type>.properties.urdf.xacro if you're simulating the robot in Gazebo. Remember to also build the host machine's workspace after editing the xacro file.

cd <host_machine_ws>
colcon build

2. Visualize the newly created URDF
2.1 Publish the URDF from the robot computer:

ros2 launch linorobot2_description description.launch.py

Optional parameters for simulation on host machine:

    rviz - Set to true to visualize the robot in rviz2 and only if you're configuring the URDF from the host machine. For example:

      ros2 launch linorobot2_description description.launch.py rviz:=true

2.2 Visualize the robot from the host machine:

The rviz argument on description.launch.py won't work on headless setup but you can visualize the robot remotely from the host machine:

ros2 launch linorobot2_viz robot_model.launch.py

Quickstart

All commands below are to be run on the robot computer unless you're running a simulation or rviz2 to visualize the robot remotely from the host machine. SLAM and Navigation launch files are the same for both real and simulated robots in Gazebo.
1. Booting up the robot
1.1a Using a real robot:

ros2 launch linorobot2_bringup bringup.launch.py

Optional parameters:

    base_serial_port - Serial port of the robot's microcontroller. The assumed value is /dev/ttyACM0. Otherwise, change the default value to the correct serial port. For example:

    ros2 launch linorobot2_bringup bringup.launch.py base_serial_port:=/dev/ttyACM1

micro_ros_baudrate - micro-ROS serial baudrate. default 115200.

ros2 launch linorobot2_bringup bringup.launch.py base_serial_port:=/dev/ttyUSB0 micro_ros_baudrate:=921600

micro_ros_transport - micro-ROS transport. default serial.

micro_ros_port - micro-ROS udp/tcp port number. default 8888.

# use micro-ROS wifi transport
ros2 launch linorobot2_bringup bringup.launch.py micro_ros_transport:=udp4 micro_ros_port:=8888

madgwick - Set to true to enable magnetometer support. The madgwick filter will fuse imu/data_raw and imu/mag to imu/data. You may visualize the IMU and manetometer by enable the IMU and magetometer plug-ins in RVIZ2. The ekf filter configuration will need update, as only 'vyaw' is enabled in the default configuration. Both IMU and magnetometer must be calibrated, otherwise the robot's pose will rotate.

# enable magnetometer support
ros2 launch linorobot2_bringup bringup.launch.py madgwick:=true orientation_stddev:=0.01

linorobot2_ws/src/linorobot2/linorobot2_base/config/ekf.yaml
    imu0: imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true, true, false]

    joy - Set to true to run the joystick node in the background. (Tested on Logitech F710).

Always wait for the microROS agent to be connected before running any application (ie. creating a map or autonomous navigation). Once connected, the agent will print:

| Root.cpp             | create_client     | create
| SessionManager.hpp   | establish_session | session established

The agent needs a few seconds to get reconnected (less than 30 seconds). Unplug and plug back in the microcontroller if it takes longer than usual.
1.1b Using Gazebo:

ros2 launch linorobot2_gazebo gazebo.launch.py

linorobot2_bringup.launch.py or gazebo.launch.py must always be run on a separate terminal before creating a map or robot navigation when working on a real robot or gazebo simulation respectively.
1.1c Using Gazebo simulation in a Docker container:

You can run gazebo in a docker container and start navigation. You must have previously built the docker image as described in the installation section above. You must cd to the docker directory to run the following commands. Press Ctrl-C to terminate a docker container.
Start gazebo container

docker compose up gazebo

Gazebo will start and display the robot in the turtlebot3-world. If you see errors related to "Unable to create rendering window", run the following command and try again:

xhost +

Start ROS navigation container

In another terminal, run

export SIM=true
docker compose up navigate

nav2 will start printing "Invalid Frame ID: map". This indicates you need to start rviz and set the 2D Pose Estimate.
Start rviz

In another terminal, run

docker compose up rviz-nav

An rviz window will open. You can set the initial position of the robot and give it goal poses to navigate to. Output to the navigate terminal shows progress from the navigation software.
Docker compose operation

In the sections above, you interactively launched containers one at a time. You can also launch multiple containers at once, running them in daemon mode with the -d flag:

export SIM=true
export RVIZ=true
docker compose up -d gazebo navigate

Shut down running containers with:

docker compose down

Note you cannot run some of the system in a docker container and other parts natively on the docker host.

You can dump logs from the ROS nodes by running

docker compose logs

2. Controlling the robot
2.1 Keyboard Teleop

Run teleop_twist_keyboard to control the robot using your keyboard:

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Press:

    i - To drive the robot forward.
    , - To reverse the robot.
    j - To rotate the robot CCW.
    l - To rotate the robot CW.
    shift + j - To strafe the robot to the left (for mecanum robots).
    shift + l - To strafe the robot to the right (for mecanum robots).
    u / o / m / . - Used for turning the robot, combining linear velocity x and angular velocity z.

2.2 Joystick

Pass joy argument to the launch file and set it to true to enable the joystick. For example:

ros2 launch linorobot2_bringup bringup.launch.py joy:=true

    On F710 Gamepad, the top switch should be set to 'X' and the 'MODE' LED should be off.

Press Button/Move Joystick:

    RB (First top right button) - Press and hold this button while moving the joysticks to enable control.
    Left Joystick Up/Down - To drive the robot forward/reverse.
    Left Joystick Left/Right - To strafe the robot to the left/right.
    Right Joystick Left/Right - To rotate the robot CW/CCW.

3. Creating a map
3.1 Run SLAM Toolbox:

ros2 launch linorobot2_navigation slam.launch.py

Optional parameters for simulation on host machine:

For example:

ros2 launch linorobot2_navigation slam.launch.py rviz:=true sim:=true

    sim - Set to true for simulated robots on the host machine. Default value is false.
    rviz - Set to true to visualize the robot in RVIZ. Default value is false.

3.1 Run rviz2 to visualize the robot from host machine:

The rviz argument on slam.launch.py won't work on headless setup but you can visualize the robot remotely from the host machine:

ros2 launch linorobot2_viz slam.launch.py

3.2 Move the robot to start mapping

Drive the robot manually until the robot has fully covered its area of operation. Alternatively, you can use the 2D Goal Pose tool in RVIZ to set an autonomous goal while mapping. More info here.
3.3 Save the map

cd linorobot2/linorobot2_navigation/maps
ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000.

4. Autonomous Navigation
4.1 Load the map you created:

Open linorobot2/linorobot2_navigation/launch/navigation.launch.py and change MAP_NAME to the name of the newly created map. Build the robot computer's workspace once done:

cd <robot_computer_ws>
colcon build

Alternatively, map argument can be used when launching Nav2 (next step) to dynamically load map files. For example:

ros2 launch linorobot2_navigation navigation.launch.py map:=<path_to_map_file>/<map_name>.yaml

4.2 Run Nav2 package:

ros2 launch linorobot2_navigation navigation.launch.py

Optional parameter for loading maps:

    map - Path to newly created map <map_name.yaml>.

Optional parameters for simulation on host machine:

    sim - Set to true for simulated robots on the host machine. Default value is false.
    rviz - Set to true to visualize the robot in RVIZ. Default value is false.

4.3 Run rviz2 to visualize the robot from host machine:

The rviz argument for navigation.launch.py won't work on headless setup but you can visualize the robot remotely from the host machine:

ros2 launch linorobot2_viz navigation.launch.py

Check out Nav2 tutorial for more details on how to initialize and send goal pose.

navigation.launch.py will continue to throw this error Timed out waiting for transform from base_link to map to become available, tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist until the robot's pose has been initialized.
