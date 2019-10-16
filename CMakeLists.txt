cmake_minimum_required (VERSION 3.1)
project (simulator)
# The version number.
set (Simulator_VERSION_MAJOR 1)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")

include_directories(includes)

# add the executable
add_executable(simulator src/main.cpp src/car_model.cpp)
 
# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
#add_library(car_model car_model.h car_model.cpp)
#target_link_libraries(simulator car_model) 
