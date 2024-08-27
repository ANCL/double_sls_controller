cmake_minimum_required(VERSION 3.0.2)
project(double_sls_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  std_msgs
  message_generation
  dynamic_reconfigure
)


include_directories("/usr/include/eigen3")


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  PTStates.msg
  ActuatorControl0.msg
  AttOut.msg
  PendAngle.msg
  DSlsState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/config.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp rospy std_msgs mavros message_runtime 
  LIBRARIES double_sls_controller
#  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs roscpp 
#  DEPENDS system_lib
   
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
)



add_library(${PROJECT_NAME}
  src/single_sls_controller.cpp
  src/double_sls_controller.cpp
)


add_dependencies(double_sls_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(offb_control_node src/offb_node.cpp)
add_executable(get_states_node src/get_states_node.cpp)
add_executable(single_sls_controller_node src/single_sls_controller_node.cpp)
add_executable(double_sls_uav0_node src/double_sls_uav0_node.cpp)
add_executable(double_sls_uav1_node src/double_sls_uav1_node.cpp)

# add_dependencies(get_states_node double_sls_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(get_states_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(offb_control_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(single_sls_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(double_sls_uav0_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(double_sls_uav1_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_subdirectory(src/StabController)
add_subdirectory(src/TracController)
add_subdirectory(src/QuadSLS_PT_Controller_QSF)

target_link_libraries(get_states_node StabController)
target_link_libraries(offb_control_node StabController)
target_link_libraries(get_states_node TracController)
target_link_libraries(offb_control_node TracController)
target_link_libraries(get_states_node QuadSLS_PT_Controller_QSF)
target_link_libraries(offb_control_node QuadSLS_PT_Controller_QSF)
target_link_libraries(single_sls_controller_node QuadSLS_PT_Controller_QSF)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_double_sls_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
