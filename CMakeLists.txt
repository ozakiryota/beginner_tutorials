cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

add_definitions(-std=c++11)	#For "std::random_devise"

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_LIBRARY(FLANN_LIBRARY flann /usr/include/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(HDF5 REQUIRED)
##### OpenMP #####
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   imudata.msg
   PointWithRange.msg
   PointWithRangeArray.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS message_runtime system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)
#include_directories(/usr/include)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/beginner_tutorials_node.cpp)

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

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(test1 src/test1.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES})
add_dependencies(test1 beginner_tutorials_generate_messages_cpp)

add_executable(test_eigen src/test_eigen.cpp)
target_link_libraries(test_eigen ${catkin_LIBRARIES})
add_dependencies(test_eigen beginner_tutorials_generate_messages_cpp)

#add_executable(kf src/kf.cpp)
#target_link_libraries(kf ${catkin_LIBRARIES})
#add_dependencies(kf beginner_tutorials_generate_messages_cpp)

add_executable(imu_test src/imu_test.cpp)
target_link_libraries(imu_test ${catkin_LIBRARIES})
add_executable(lidar_sub src/lidar_sub.cpp)
target_link_libraries(lidar_sub ${catkin_LIBRARIES})
add_executable(test_pcl_view src/test_pcl_view.cpp)
target_link_libraries(test_pcl_view ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_pcl_norm src/test_pcl_norm.cpp)
target_link_libraries(test_pcl_norm ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_pcl_kdtree src/test_pcl_kdtree.cpp)
target_link_libraries(test_pcl_kdtree ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_pcl_randomcloud src/test_pcl_randomcloud.cpp)
target_link_libraries(test_pcl_randomcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_pcl_dkanmap src/test_pcl_dkanmap.cpp)
target_link_libraries(test_pcl_dkanmap ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_pcl_bag src/test_pcl_bag.cpp)
target_link_libraries(test_pcl_bag ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(sq2_normals src/sq2_normals.cpp)
target_link_libraries(sq2_normals ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(imu_tf src/imu_tf.cpp)
target_link_libraries(imu_tf ${catkin_LIBRARIES})
# add_executable(ekf src/ekf.cpp)
# target_link_libraries(ekf ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(imu_alignment src/imu_alignment.cpp)
target_link_libraries(imu_alignment ${catkin_LIBRARIES})
add_executable(imu_odom src/imu_odom.cpp)
target_link_libraries(imu_odom ${catkin_LIBRARIES})
add_executable(imu_alignment_nomove src/imu_alignment_nomove.cpp)
target_link_libraries(imu_alignment_nomove ${catkin_LIBRARIES})
add_executable(imu_alignment0 src/imu_alignment0.cpp)
target_link_libraries(imu_alignment0 ${catkin_LIBRARIES})
add_executable(imu_alignment_ src/imu_alignment_.cpp)
target_link_libraries(imu_alignment_ ${catkin_LIBRARIES})
add_executable(tf_from_posemsg src/tf_from_posemsg.cpp)
target_link_libraries(tf_from_posemsg ${catkin_LIBRARIES})
add_executable(imu_alignment_simple src/imu_alignment_simple.cpp)
target_link_libraries(imu_alignment_simple ${catkin_LIBRARIES})
add_executable(kf_position src/kf_position.cpp)
target_link_libraries(kf_position ${catkin_LIBRARIES})
add_executable(tf_lsdslam src/tf_lsdslam.cpp)
target_link_libraries(tf_lsdslam ${catkin_LIBRARIES})
add_executable(ekf_imu_slam src/ekf_imu_slam.cpp)
target_link_libraries(ekf_imu_slam ${catkin_LIBRARIES})
add_executable(odom2dto3d src/odom2dto3d.cpp)
target_link_libraries(odom2dto3d ${catkin_LIBRARIES})
add_executable(odom2dto3d_ src/odom2dto3d_.cpp)
target_link_libraries(odom2dto3d_ ${catkin_LIBRARIES})
add_executable(lsdodom src/lsdodom.cpp)
target_link_libraries(lsdodom ${catkin_LIBRARIES})
# add_executable(imu_alignment_bias src/imu_alignment_bias.cpp)
# target_link_libraries(imu_alignment_bias ${catkin_LIBRARIES})
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_executable(dualekf src/dualekf.cpp)
target_link_libraries(dualekf ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(odom3d_with_posemsg src/odom3d_with_posemsg.cpp)
target_link_libraries(odom3d_with_posemsg ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_executable(gyrodometry3d src/gyrodometry3d.cpp)
# target_link_libraries(gyrodometry3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(lsd_odom src/lsd_odom.cpp)
target_link_libraries(lsd_odom ${catkin_LIBRARIES})
add_executable(velodyne_fitting_walls src/velodyne_fitting_walls.cpp)
target_link_libraries(velodyne_fitting_walls ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(quaternion_convertor src/quaternion_convertor.cpp)
target_link_libraries(quaternion_convertor ${catkin_LIBRARIES})
add_executable(odom_printer_ src/odom_printer_.cpp)
target_link_libraries(odom_printer_ ${catkin_LIBRARIES})
add_executable(velodyne_odom src/velodyne_odom.cpp)
target_link_libraries(velodyne_odom ${catkin_LIBRARIES})
add_executable(pc_normals src/pc_normals.cpp)
target_link_libraries(pc_normals ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_least_squares_method src/test_least_squares_method.cpp)
target_link_libraries(test_least_squares_method ${catkin_LIBRARIES} ${PCL_LIBRARIES})
find_package(kdl_conversions)
add_executable(ndt src/ndt.cpp)
target_link_libraries(ndt ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(ndt_ src/ndt_.cpp)
target_link_libraries(ndt_ ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_ndt src/test_ndt.cpp)
target_link_libraries(test_ndt ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(icp src/icp.cpp)
target_link_libraries(icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(test_for_loop src/test_for_loop.cpp)
target_link_libraries(test_for_loop ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(euclidean_clustering_test src/euclidean_clustering_test.cpp)
target_link_libraries(euclidean_clustering_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_dependencies(imu_alignment_ beginner_tutorials_generate_messages_cpp)
add_dependencies(imu_alignment_nomove beginner_tutorials_generate_messages_cpp)
add_dependencies(imu_alignment_simple beginner_tutorials_generate_messages_cpp)
add_dependencies(imu_alignment beginner_tutorials_generate_messages_cpp)
