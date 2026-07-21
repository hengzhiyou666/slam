cmake_minimum_required(VERSION 3.5)
project(icp_relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wextra -Wpedantic)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_BUILD_TYPE Release)

option(USE_LIVOX "Build Livox CustomMsg support" OFF)
option(BUILD_SAC_IA_GICP "Build the optional SAC-IA/GICP executable" ON)
# define use_livox
if(USE_LIVOX)
  add_definitions(-DUSE_LIVOX)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

set(OMNI_SLAM_DEPS_ROOT "" CACHE PATH "Private dependency root used on dog3")
if(OMNI_SLAM_DEPS_ROOT)
  set(PCL_INCLUDE_DIRS
    "${OMNI_SLAM_DEPS_ROOT}/usr/include/pcl-1.12"
    "${OMNI_SLAM_DEPS_ROOT}/usr/include"
  )
  set(OMNI_PCL_LIBRARY_DIR "${OMNI_SLAM_DEPS_ROOT}/usr/lib/aarch64-linux-gnu")
  set(PCL_LIBRARIES)
  foreach(PCL_COMPONENT registration features filters sample_consensus search kdtree octree common)
    find_library(PCL_${PCL_COMPONENT}_LIBRARY pcl_${PCL_COMPONENT}
      PATHS "${OMNI_PCL_LIBRARY_DIR}" NO_DEFAULT_PATH REQUIRED)
    list(APPEND PCL_LIBRARIES ${PCL_${PCL_COMPONENT}_LIBRARY})
  endforeach()
else()
  set(ICP_PCL_COMPONENTS common filters registration)
  if(BUILD_SAC_IA_GICP)
    list(APPEND ICP_PCL_COMPONENTS io)
  endif()
  find_package(PCL REQUIRED COMPONENTS ${ICP_PCL_COMPONENTS})
endif()
if(USE_LIVOX)
  find_package(livox_ros_driver2 REQUIRED)
endif()

include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)

add_executable(icp_node src/icp_node.cpp)
ament_target_dependencies(
  icp_node rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros
  tf2_geometry_msgs)
target_link_libraries(icp_node ${PCL_LIBRARIES})
if(OMNI_SLAM_DEPS_ROOT)
  set_target_properties(icp_node PROPERTIES
    BUILD_RPATH "${OMNI_PCL_LIBRARY_DIR}"
    INSTALL_RPATH "${OMNI_PCL_LIBRARY_DIR}")
endif()
if(USE_LIVOX)
  ament_target_dependencies(icp_node livox_ros_driver2)
endif()

add_executable(transform_publisher src/transform_publisher.cpp)
ament_target_dependencies(
  transform_publisher
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(transform_publisher Eigen3::Eigen)


if(BUILD_SAC_IA_GICP)
  add_executable(sac_ia_gicp src/sac_ia_gicp.cpp)
  ament_target_dependencies(sac_ia_gicp rclcpp std_msgs sensor_msgs)
  target_link_libraries(sac_ia_gicp ${PCL_LIBRARIES} Eigen3::Eigen)
endif()

# Install launch files
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
set(ICP_TARGETS icp_node transform_publisher)
if(BUILD_SAC_IA_GICP)
  list(APPEND ICP_TARGETS sac_ia_gicp)
endif()
install(TARGETS ${ICP_TARGETS} DESTINATION lib/${PROJECT_NAME})

ament_package()
