cmake_minimum_required(VERSION 2.8.3)
project(timon_demo)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             interactive_markers
             moveit_visual_tools 
             moveit_ros_perception
             geometric_shapes
             pcl_ros
             pcl_conversions
             rosbag
             tf2_ros
             tf2_eigen
             tf2_geometry_msgs
            )
  

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(push_button src/push_button.cpp)
target_link_libraries(push_button ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS push_button DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(push_button_2 src/push_button_2.cpp)
target_link_libraries(push_button_2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS push_button_2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(push_button_3 src/push_button_3.cpp)
target_link_libraries(push_button_3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS push_button_3 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(push_button_4 src/push_button_4.cpp)
target_link_libraries(push_button_4 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS push_button_4 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS tf_listener DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(add_objects src/add_objects.cpp)
target_link_libraries(add_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS add_objects DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(joint_tests src/joint_tests.cpp)
target_link_libraries(joint_tests ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS joint_tests DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_timon_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
