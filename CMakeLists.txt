cmake_minimum_required(VERSION 3.10)
project(hhas)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the build type to release
set(CMAKE_BUILD_TYPE Release)

# Add the directory where AStar and GridAStar headers are stored
include_directories(include)

# find OpenCV
find_package(OpenCV REQUIRED)

#find nanoFLANN
find_package(Threads REQUIRED)
find_package(nanoflann REQUIRED)

#find omp
find_package(OpenMP REQUIRED)

# Define the executable to be built (named grid_a_star) and the source file for it
add_executable(grid_a_star src/grid_a_star.cpp)

add_executable(se2_grid_a_star src/se2_grid_a_star.cpp)

add_executable(se2_grid_a_star_fp src/se2_grid_a_star_fp.cpp)

# link OpenCV to se2_grid_a_star target
target_link_libraries(se2_grid_a_star ${OpenCV_LIBS})

# link opencv and nanoflann to se2_grid_a_star_fp target, also link OpenMP
target_link_libraries(se2_grid_a_star_fp ${OpenCV_LIBS} nanoflann::nanoflann OpenMP::OpenMP_CXX)