cmake_minimum_required(VERSION 3.8)
project(Rail_Lidar_Segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization features keypoints)
find_package(Eigen3 REQUIRED)

# Inclure les répertoires de PCL
add_definitions(${PCL_DEFINITIONS})
include_directories(
    ${PCL_INCLUDE_DIRS}
    -I/usr/include/pcl-1.12
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable (railway_segmentation src/railway_segmentation.cpp)



target_link_libraries (railway_segmentation ${PCL_LIBRARIES})