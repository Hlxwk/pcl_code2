cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(loadPCD)

add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(include)

add_library(loadPCD
  src/voxel_grid_covariance_omp.cpp
  src/ndt_omp.cpp
  src/gicp_omp.cpp
)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (load apps/loadPCD.cpp)

add_dependencies(load
  loadPCD
)

target_link_libraries (load ${PCL_LIBRARIES} ${LIBS} -llas loadPCD)
