<img src="https://i.ibb.co/Yfj2n1k/jerotimon-logo.png" align="center" width="600" alt="header pic"/>

<!-- # BIR 2.0 CHALLENGE - JeRoTIMON Manipulator -->


JeRoTIMON is a manipulator that is being designed, simulated and built in order to meet demands related to the recognition of visual markers and in response to actuate switches, keys or buttons. These buttons and markers can be viewed horizontally or vertically.

<center><img src="https://i.ibb.co/W3psqkZ/Screenshot-from-2020-10-15-17-49-09.png" class="centerImage"></center>


## Open Source software and packages related
### Software
- [ROS Melodic](http://wiki.ros.org/melodic)
- [MoveIt](https://moveit.ros.org/)
### Packages
- [BIR Marker](https://github.com/Brazilian-Institute-of-Robotics/bir_marker_localization/tree/final_settings)
- [def_cam_teledyne_nano](https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano) 
- [OpenCV 3](https://opencv.org/)

# User manual
## Simulate JeRoTIMON
<img src="https://i.ibb.co/CQBpPpz/simugif.gif" class="centerImage">

### Install softwares and libraries:
- [Install ROS Melodic on Ubuntu 18.04](https://www.learnopencv.com/install-opencv3-on-ubuntu/)
- [Intall OpenCV 3.3.1](https://www.learnopencv.com/install-opencv3-on-ubuntu/)
- [Install MoveIt](https://moveit.ros.org/install)

### Install ROS melodic packages

`$ sudo apt install ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-controller-manager ros-melodic-joint-trajectory-controller ros-melodic-joint-state-controller ros-melodic-position-controllers ros-melodic-trac-ik-kinematics-plugin`

`$ sudo apt-get install ros-melodic-moveit-visual-tools`

### Setup workspace
```
$ mkdir -p catkin_ws/src
$ cd  catkin_ws/src 

$ git clone -b feature/simulation https://github.com/Brazilian-Institute-of-Robotics/timon_hm_manipulator.git
$ git clone -b final_settings https://github.com/Brazilian-Institute-of-Robotics/bir_marker_localization.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git

$ cd ..

$ catkin_make
```
### Run application
#### Gazebo simulation
`$ roslaunch manipulator_gazebo gazebo.launch`

#### Moveit 
`$ roslaunch manipulator_gazebo moveit_demo.launch`

#### Marker localization
`$ roslaunch timon_demo bir_marker_localization.launch`

#### Execute mission
`$ roslaunch timon_demo push_button_simulation.launch`

### How to change a box orientation
For change the box orientation, go to *manipulator_gazebo/launch/spawn_box.launch* and change *box_vertical.urdf.xacro* for *box_horizontal.urdf.xacro*.

## Operate JeRoTIMON manipulator
<img src="https://i.ibb.co/RTsbHtB/realgif.gif" class="centerImage" width="624">

Install the same software and packages than simulation and download and install the DALSA framework for Ubuntu. You need to sign up on their website to download it.

### Setup workspace
```
$ mkdir -p catkin_ws/src
$ cd  catkin_ws/src 

$ git clone https://github.com/Brazilian-Institute-of-Robotics/timon_hm_manipulator.git
$ git clone -b final_settings https://github.com/Brazilian-Institute-of-Robotics/bir_marker_localization.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone -b refactor_code https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano.git
```
Copy [this package](https://github.com/ROBOTIS-GIT/dynamixel-workbench/tree/feature-moveit-bridge/dynamixel_workbench_moveit_bridge) to dynamixel-workbench folder. 

```
$ cd ..

$ catkin_make
```
### Run application
#### Controllers
`$ roslaunch timon_arm_controller dxl_controllers.launch`

`$ roslaunch timon_arm_controller moveit.launch`

`$ roslaunch timon_arm_controller dxl_moveit_bridge.launch`

#### Camera
`$ roslaunch def_cam_teledyne_nano camera_example.launch`

#### Marker localization
`$ roslaunch timon_demo bir_marker_localization.launch`

#### Execute mission
`$ roslaunch timon_demo push_button_real.launch`


# JeRoTIMON specifications

## Specifications table
| Item  |  JeRoTIMON  |
| :---: | :---: |
|  DOF | 5 |
| Payload| 2 kg|
|Reach| 981 mm|
|Weight (with base) | 10 kg|
|Weight (without base) | 6.4 kg|
|Operating voltage | 24 V |
|Resolution| Joint 1, Joint 2, Joint 3, Joint 4: 1,003,846 pulse/rev |
||Joint 5: 607,500 pulse/rev|
|Motors| Joint 1, Joint 2, Joint 3 : PH54-200-S500-R (200W) |
| | Joint 4: PH54-100-S500-R (100 W) |
| | Joint 5: PH42-020-S300-R (20 W) |
|Operating Range| Joint 1: -45°  ~  45°|
| | Joint 2: -90°  ~  90° |
| | Joint 3: -43°  ~  173° |
| | Joint 4: -90°  ~  90° |
| | Joint 5: -90°  ~  90° |
|Camera| Teledyne Genie Nano C290 |
|Position sensor type| Homing: Absolute Encoder |
| | Control: Incremental Encoder  |
|Communication | USB |
|Electrical pattern | RS485 |
|Communication Baudrate |  57,600 bps |

## Dimension
<img src="https://i.ibb.co/VvmGbqQ/dimensions.png">

## Home Position
<img src="https://i.ibb.co/ypbScLn/manipulator-home.jpg">

## Workspace
### XY
<img src="https://i.ibb.co/McfSktY/workspace-XY.png">

### XZ
<img src="https://i.ibb.co/zb2L9Zn/workspace-XZ.png">

### YZ
<img src="https://i.ibb.co/HX1GPFy/workspace-YZ.png">

## DH Configuration
<img src="https://i.ibb.co/P1BK3Xh/dh-configuration.png">

### DH parameters

| LINK | Link Lenght (mm) | Link Twist (rad) | Joint Offset (mm) | DXL Angle (rad) |
|:---:|:---:|:---:|:---:|:---:|
|1|0|π/2|12|0|
|2|452||0|<img src="https://i.ibb.co/Qrbhxsk/eaq1.png" width="70%">|
|3|30|-π/2|0|<img src="https://i.ibb.co/Yd8bTp3/eq2.png" width="70%">|
|4|0|π/2|491|0|
|5|0|0|0|0|

## Mass Property
### Coordinate
<img src="https://i.ibb.co/Dtcgfrs/coordinate.jpg">

### Link 0
<img src="https://i.ibb.co/r5j3Dnn/link0.jpg">

- **Mass:** 3.72384654 kg
- **Volume:** 0.00413037 m³
- **Surface area:** 0.28622123 m²
- **Center of mass:**
  - X: -0.00000147 m
  - Y: 0.00000000 m
  - Z: 0.03504069 m
- **Moments of inertia: kg m²**
  - **Lxx:** 0.01072644 **Lxy:** -9.465e-9 **Lxz:** -3.247e-8
  - **Lyx:** -9.465e-9 **Lyy:** 0.04865651 **Lyz:** 6.830e-13
  - **Lzx:** -3.247e-8 **Lzy:** 6.830e-13 **Lzz:** 0.05388213
### Link 1
<img src="https://i.ibb.co/GxVBZkM/link1.jpg">

- **Mass:** 1.03781084 kg
- **Volume:** 0.00040169 m³
- **Surface area:** 0.05853078 m²
- **Center of mass:**
  - X: 0.00814457 m
  - Y: 4.45597047e − 8 m
  - Z: 0.16022275 m
- **Moments of inertia: kg m²**
  - **Lxx:** 0.0005687 **Lxy:** 2.602e-10 **Lxz:** -0.00004027
  - **Lyx:** 2.602e-10 **Lyy:** 0.00166759 **Lyz:** -2.222e-10
  - **Lzx:** -0.00004027 **Lzy:** -2.222e-10 **Lzz:** 0.00155695
### Link 2
<img src="https://i.ibb.co/w0YmhvZ/link2.jpg">

- **Mass:** 2.05026959 kg
- **Volume:** 0.00077667 m³
- **Surface area:** 0.29790369 m²
- **Center of mass:**
  - X: 0.00294129 m
  - Y: −0.01058166 m
  - Z: 0.48960322 m
- **Moments of inertia: kg m²**
  - **Lxx:** 0.06174738 **Lxy:** -0.00003476 **Lxz:** 0.00098626
  - **Lyx:** -0.00003476 **Lyy:** 0.06319525 **Lyz:** 0.00311701
  - **Lzx:** 0.00098626 **Lzy:** 0.00311701 **Lzz:** 0.0034029

### Link 3
<img src="https://i.ibb.co/0ZNpZfW/link3.jpg">

- **Mass:** 1.83580505 kg
- **Volume:** 0.00081683 m³
- **Surface area:** 0.32055793 m²
- **Center of mass:**
  - X: 0.00257384 m
  - Y: 0.00207951 m
  - Z: 0.91287054 m
- **Moments of inertia: kg m²**
  - **Lxx:** 0.03439469 **Lxy:** -0.00000261 **Lxz:** -0.00001194
  - **Lyx:** -0.00000261 **Lyy:** 0.03489631 **Lyz:** -0.0001288
  - **Lzx:** -0.00001194 **Lzy:** -0.0001288 **Lzz:** 0.0023687
### Link 4
<img src="https://i.ibb.co/JstQ8Cn/link4.jpg">

- **Mass:** 0.39425655 kg
- **Volume:** 0.00014602 m³
- **Surface area:** 0.03018022 m²
- **Center of mass:**
  - X: 0.00257156 m
  - Y: 0.00133073 m
  - Z: 1.09673426 m
- **Moments of inertia: kg m²**
  - **Lxx:** 0.00028795 **Lxy:** 2.010e-10 **Lxz:** -8.166e-13
  - **Lyx:** 2.010e-10 **Lyy:** 0.00011981 **Lyz:** -0.00000446
  - **Lzx:** -8.166e-13 **Lzy:** -0.00000446 **Lzz:** 0.0002842
### Link 5
<img src="https://i.ibb.co/GJ7CJDZ/link5.jpg">

- **Mass:** 0.08694786 kg
- **Volume:** 0.0000322 m³
- **Surface area:** 0.02435944 m²
- **Center of mass:**
  - X: 0.00257155 m
  - Y: 0.00684212 m
  - Z: 1.13578562 m
- **Moments of inertia: kg m²**
  - **Lxx:** 0.00012963 **Lxy:** -5.235e-13 **Lxz:** -9.996e-14
  - **Lyx:** -5.235e-13 **Lyy:** 0.00003703 **Lyz:** 0.00000247
  - **Lzx:** -9.996e-14 **Lzy:** 0.00000247 **Lzz:** 0.00012613

