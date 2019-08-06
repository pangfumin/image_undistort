cmake_minimum_required(VERSION 2.8.3)

project(image_undistort)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Set compile flags (-Ofast actually makes a big difference over -O3 here (maybe 20% faster)
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  sensor_msgs
  cv_bridge
  image_transport
  tf_conversions
  nodelet
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
)

find_package(OpenCV REQUIRED COMPONENTS core calib3d)

find_package(NLOPT)
IF(${NLOPT_FOUND})
  include_directories(
    ${NLOPT_INCLUDE_DIRS}
  )

  add_library(${PROJECT_NAME}_nodelet 
  src/nodelets/image_undistort_nodelet.cpp 
  src/nodelets/stereo_info_nodelet.cpp
  src/nodelets/stereo_undistort_nodelet.cpp
  src/nodelets/depth_nodelet.cpp
  src/nodelets/point_to_bearing_nodelet.cpp
  src/stereo_info.cpp
  src/stereo_undistort.cpp
  src/depth.cpp
  src/camera_parameters.cpp 
  src/image_undistort.cpp 
  src/undistorter.cpp
  src/point_to_bearing.cpp
          src/ros-free/stereo_dense.cpp)

ELSE()

  add_library(${PROJECT_NAME}_nodelet 
  src/nodelets/image_undistort_nodelet.cpp 
  src/nodelets/stereo_info_nodelet.cpp
  src/nodelets/stereo_undistort_nodelet.cpp
  src/nodelets/depth_nodelet.cpp
  src/stereo_info.cpp
  src/stereo_undistort.cpp
  src/depth.cpp
  src/camera_parameters.cpp 
  src/image_undistort.cpp 
  src/undistorter.cpp
          src/ros-free/stereo_dense.cpp)

  MESSAGE(WARNING "nlopt not found, building without point to bearing nodelet")
ENDIF()

add_library(${PROJECT_NAME}_lib
        src/stereo_info.cpp
        src/stereo_undistort.cpp
        src/depth.cpp
        src/camera_parameters.cpp
        src/image_undistort.cpp
        src/undistorter.cpp
#        src/point_to_bearing.cpp
        src/ros-free/stereo_dense.cpp
        src/ros-free/file-system-tools.cpp
        )


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_lib
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
        )


add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})

add_executable(image_undistort_node src/nodes/image_undistort_node.cpp)
target_link_libraries(image_undistort_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)



add_executable(stereo_info_node src/nodes/stereo_info_node.cpp)
target_link_libraries(stereo_info_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(stereo_undistort_node src/nodes/stereo_undistort_node.cpp)
target_link_libraries(stereo_undistort_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)



add_executable(depth_node src/nodes/depth_node.cpp)
target_link_libraries(depth_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(dense_stereo_node src/nodes/dense_stereo_node.cpp)
target_link_libraries(dense_stereo_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)



add_executable(stereo_dense_node src/ros-free/stereo_dense_node.cpp)
target_link_libraries(stereo_dense_node
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}
        ${PROJECT_NAME}_lib
        )

add_executable(image_undistort_node_nonros src/ros-free/image_undistort_node.cpp)
target_link_libraries(image_undistort_node_nonros
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}
        ${PROJECT_NAME}_lib
        )

add_executable(mono_undistort_node src/ros-free/mono_undistort_node.cpp)
target_link_libraries(mono_undistort_node
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}
        ${PROJECT_NAME}_lib
        )


add_executable(stereo_dense_MultiVehicleStereo_node src/ros-free/stereo_dense_MultiVehicleStereo_node.cpp)
target_link_libraries(stereo_dense_MultiVehicleStereo_node
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}
        ${PROJECT_NAME}_lib
        )


add_executable(stereo_calib_node src/ros-free/stereo_calib.cpp)
target_link_libraries(stereo_calib_node
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}
        ${PROJECT_NAME}_lib
        )

add_executable(tool_list_image src/ros-free/tool_list_image.cpp)
target_link_libraries(tool_list_image
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}
        ${PROJECT_NAME}_lib
        )

IF(${NLOPT_FOUND})
  add_executable(point_to_bearing_node src/nodes/point_to_bearing_node.cpp)
  target_link_libraries(point_to_bearing_node
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${NLOPT_LIBRARIES}
  )
ENDIF()

# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

