cmake_minimum_required(VERSION 3.5)
project(jason)

find_package(osqp REQUIRED)
find_package(mujoco REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 3.3 REQUIRED)
# find_package(orlqp REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

#add_library(jason SHARED)
#target_sources(jason PRIVATE
    # src/my_file.cpp
#)
#target_include_directories(jason PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add the executable
add_executable(main src/main.cpp)
target_include_directories(main PUBLIC include)
target_link_libraries(main PUBLIC stdc++ stdc++fs m osqp::osqp mujoco::mujoco glfw) 
# target_link_libraries(main PUBLIC stdc++ stdc++fs m osqp::osqp mujoco::mujoco orlqp::orlqp glfw)