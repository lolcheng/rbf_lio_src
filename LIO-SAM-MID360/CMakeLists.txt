cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

# Livox support is optional. M20/Airy only needs sensor_msgs/PointCloud2.
find_package(livox_ros_driver QUIET)
find_package(livox_ros_driver2 QUIET)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(CUDA QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  M20JointsData.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${livox_ros_driver_INCLUDE_DIRS}
	${livox_ros_driver2_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${livox_ros_driver_LIBRARIES} ${livox_ros_driver2_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization
  src/mapOptmization.cpp
  src/rbf_wheel_kinematics.cpp
  src/rbf_cuda_wheel_gradient.cpp
)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
if(CUDA_FOUND)
  add_definitions(-DLIO_SAM_WITH_CUDA)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++14)
  cuda_add_library(${PROJECT_NAME}_rbf_cuda src/cuda/rbf_cuda_wheel_gradient.cu)
  target_link_libraries(${PROJECT_NAME}_mapOptmization ${PROJECT_NAME}_rbf_cuda)
endif()
target_link_libraries(${PROJECT_NAME}_mapOptmization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ${CUDA_LIBRARIES})

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

catkin_install_python(PROGRAMS
  scripts/joint_state_name_adapter.py
  scripts/m20_joint_state_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
