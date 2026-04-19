cmake_minimum_required(VERSION 3.8)
project(Rail_Lidar_Segmentation)

# --- 1. CONFIGURATION DU COMPILATEUR ---

set(Boost_NO_BOOST_CMAKE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ajout des drapeaux d'optimisation pour le mode Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 2. RECHERCHE DES DÉPENDANCES ---
# On regroupe tout en un seul appel propre
# Aide PCL à trouver Boost sur Ubuntu
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(PDAL REQUIRED)

# B. EIGEN
find_package(Eigen3 REQUIRED)



set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL REQUIRED COMPONENTS common io visualization filters search features segmentation)

# On ajoute explicitement les répertoires d'inclusion de Boost au cas où
include_directories(${Boost_INCLUDE_DIRS})

# Inclure les répertoires de PCL
add_definitions(${PCL_DEFINITIONS})
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)





include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable (railway_segmentation src/railway_segmentation.cpp)

add_executable (test_segmentation src/test.cpp)

target_link_libraries (railway_segmentation ${PCL_LIBRARIES} ${PDAL_LIBRARIES} Eigen3::Eigen)
target_link_libraries (test_segmentation ${PCL_LIBRARIES} ${PDAL_LIBRARIES} Eigen3::Eigen)