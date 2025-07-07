cmake_minimum_required(VERSION 3.0.0)
project(csir_franka_control VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(main main.cpp)

set (CMAKE_CXX_STANDARD 11)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};$ENV{HOME}/local")
include(CPack)


#find essentiao package
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)

include_directories(Inc)

#compile library
add_subdirectory(Src)
add_subdirectory(Test)
# add_executable(my_app main.cpp)

#### hanxi changed
set(SOURCES
  main.cpp
  Src/threads.cc
  Src/robot.cc
  Src/udp.cc
  Src/examples_common.cc
  # other.cc
)

add_executable(my_app ${SOURCES})

target_include_directories(my_app
  PRIVATE
    $ENV{HOME}/local/include/eigen3
)
####

target_link_libraries(my_app PRIVATE Eigen3::Eigen Threads::Threads threads Franka::Franka )

#
target_link_libraries(main Threads::Threads threads)




