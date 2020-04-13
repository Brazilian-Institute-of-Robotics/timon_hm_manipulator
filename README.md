# Timon-HM Manipulator
<center><img src="https://i.ibb.co/M2nPsrL/readme.jpg" class="centerImage"></center>

Timon-HM is a manipulator that is being designed, simulated and built in order to meet demands related to the recognition of visual markers and in response to actuate switches, keys or buttons.

## Open Source software and packages related to Timon-HM manipulator
### Software
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [MoveIt](https://moveit.ros.org/install)
### Packages
- ros-melodic-ros-control 
- ros-melodic-gazebo-ros-control 
- ros-melodic-controller-manager 
- ros-melodic-joint-trajectory-controller 
- ros-melodic-joint-state-controller
- ros-melodic-position-controllers
- ros-melodic-trac-ik-kinematics-plugin
- [BIR Marker](https://github.com/Brazilian-Institute-of-Robotics/bir_marker_localization/tree/final_ajusts)

## Launchers
### Gazebo:
`$ roslaunch manipulator_gazebo gazebo.launch`

### Timon-HM MoveIt package:
`$ roslaunch manipulator_gazebo moveit_demo.launch`

### Search the visual marker:
`$ roslaunch timon_demo  motion_plan5.launch`

# Timon-HM specifications

## Specifications table
| Item  |  Timon-HM  |
| :---: | :---: |
|  DOF | 5 |
| Payload| 1.94 kg|
|Reach| 979 mm|
|Weight| 7.38 kg|
|Operating voltage | 24 V |
|Resolution| Joint 1, Joinnt 2: 4096 pulse/rev |
|| Joint 3, Joint 4, Joint 5: 1,003,846 pulse/rev|
|Motors| Joint 1, Joint 2: H54-200-S500-R (200W) |
| | Joint 3(2), Joint 4, Joint 5: MX-106 (65W) |
|Operating Range| Joint 1: -π/2 (rad)  ~  π/2 (rad)|
| | Joint 2: -π/2(rad)  ~  π/2 (rad) |
| | Joint 3: -π/2(rad)  ~  3π/4 (rad) |
| | Joint 4: -π(rad)  ~  π (rad) |
| | Joint 5: -π/2(rad)  ~  π/2 (rad) |
|Camera| Teledyne Genie Nano |
|Position sensor type| Homing: Absolute Encoder |
| | Control: Incremental Encoder  |
|Communications | RS485 |
|Communication Baudrate |  1 Mbps |

## Dimension
<img src="https://i.ibb.co/TK2LZQf/planta-2-D.jpg">

## Home Position
<img src="https://i.ibb.co/VTdpTWw/begin-timon.png">

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
<img src="https://i.ibb.co/ZSYPGyd/coordinates.jpg">

### Link 0
<img src="https://i.ibb.co/kMBGPZ4/link0.jpg">

- **Mass:** 3.72264467 kg
- **Volume:** 0.00412764 m³
- **Surface area:** 0.28500950 m²
- **Center of mass:**
  - X: -0.00000012 m
  - Y: 0.00000000 m
  - Z: 0.03490035 m
- **Moments of inertia: kg m²**
  - **Lxx:** 1.06821989e-2 **Lxy:** -2.59829620e-6 **Lxz:** 1.68828926e-8
  - **Lyx:** -2.59829620e-6 **Lyy:** 4.86112355e-2 **Lyz:** 0.00000000e+0
  - **Lzx:** 1.68828926e-8 **Lzy:** 0.00000000e+0 **Lzz:** 5.38837109e-2
### Link 1
<img src="https://i.ibb.co/PtcjdDy/link1.jpg">

- **Mass:** 1.00643107 kg
- **Volume:** 0.00040209 m³
- **Surface area:** 0.05733417 m²
- **Center of mass:**
  - X: 0.01039500 m
  - Y: 0.00005146 m
  - Z: 0.16006794 m
- **Moments of inertia: kg m²**
  - **Lxx:** 5.52840983e-4 **Lxy:** -4.82304098e-6 **Lxz:** -5.15873870e-5
  - **Lyx:** -4.82304098e-6 **Lyy:** 1.52709797e-3 **Lyz:** -2.55376135e-7
  - **Lzx:** -5.15873870e-5 **Lzy:** -2.55376135e-7 **Lzz:** 1.41817064e-3
### Link 2
<img src="https://i.ibb.co/gJgfX27/link2.jpg">

- **Mass:** 1.43140941 kg
- **Volume:** 0.00072217 m³
- **Surface area:** 0.30338280 m²
- **Center of mass:**
  - X: 0.00402379 m
  - Y: -0.00489519 m
  - Z: 0.42191352 m
- **Moments of inertia: kg m²**
  - **Lxx:** 4.01177388e-2 **Lxy:** -1.99033225e-6 **Lxz:** 7.25766892e-4
  - **Lyx:** -1.99033225e-5 **Lyy:** 4.07927622e-2 **Lyz:** 1.35014731e-3
  - **Lzx:** 7.25766892e-4 **Lzy:** 1.35014731e-3 **Lzz:** 1.93910351e-3
### Link 3
<img src="https://i.ibb.co/mCYG8wc/link3.jpg">

- **Mass:** 1.02430185 kg
- **Volume:** 0.00048651 m³
- **Surface area:** 0.27643743 m²
- **Center of mass:**
  - X: 0.00177442 m
  - Y: -0.02543863 m
  - Z: 0.88744337 m
- **Moments of inertia: kg m²**
  - **Lxx:** 2.15420518e-2 **Lxy:** 4.48113214e-6 **Lxz:** 7.29467793e-6
  - **Lyx:** 4.48113214e-6 **Lyy:** 2.09581064e-2 **Lyz:** -9.20042318e-4
  - **Lzx:** 7.29467793e-6 **Lzy:** -9.20042318e-4 **Lzz:** 1.37743730e-3
### Link 4
<img src="https://i.ibb.co/BTTmnhJ/link4.jpg">

- **Mass:** 0.17387384 kg
- **Volume:** 0.00008979 m³
- **Surface area:** 0.02133374 m²
- **Center of mass:**
  - X: 0.00204423 m
  - Y: -0.03416008 m
  - Z: 1.09140950 m
- **Moments of inertia: kg m²**
  - **Lxx:** 7.27594009e-5 **Lxy:** 3.60335785e-7 **Lxz:** -1.43417976e-6
  - **Lyx:** 3.60335785e-7 **Lyy:** 9.11402947e-5 **Lyz:** 1.03937291e-8
  - **Lzx:** -1.43417976e-6 **Lzy:** 1.03937291e-8 **Lzz:** 4.72120463e-5
### Link 5
<img src="https://i.ibb.co/hcXLvhr/link5.jpg">

- **Mass:** 0.01873963 kg
- **Volume:** 0.00000694 m³
- **Surface area:** 0.00883439 m²
- **Center of mass:**
  - X: 0.00184227 m
  - Y: -0.03451643 m
  - Z: 1.13452531 m
- **Moments of inertia: kg m²**
  - **Lxx:** 4.49832477e-6 **Lxy:** 1.27964175e-7 **Lxz:** 9.56680130e-10
  - **Lyx:** 1.27964175e-7 **Lyy:** 1.12015451e-5 **Lyz:** 4.61798902e-9
  - **Lzx:** 9.56680130e-10 **Lzy:** 4.61798902e-9 **Lzz:** 1.08114418e-5

