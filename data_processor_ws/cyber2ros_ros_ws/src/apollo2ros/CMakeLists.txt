cmake_minimum_required(VERSION 3.10.2)
project(apollo2ros LANGUAGES CXX VERSION 0.0.1)

# -fPIC：生成动态库，-fopenmp 开启多线程，-O3 对代码进行优化，-g 打印调试信息，-Wall 打印所有警告信息, pthread 支持多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -g -O3 -fopenmp -pthread -Wall")

# 不生成警告信息: debug阶段尽量不要使用
add_definitions(-w)

find_package(Boost REQUIRED COMPONENTS thread filesystem program_options system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_generation
    std_srvs
    cv_bridge
    image_transport
    roslib
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)


catkin_package(
    CATKIN_DEPENDS
    message_runtime
    message_generation
    roscpp
    sensor_msgs
    cv_bridge
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    /usr/local/include/eigen3
)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin)

# add_library(${PROJECT_NAME}_lib SHARED
# )

#必须为绝对路径，如有多个文件使用空格隔开
set(EZCFG_STRUCT_HEADER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/config.h)

add_subdirectory(third_party/easy_config)
add_subdirectory(common)
add_subdirectory(interface)

add_executable(main main.cpp)
target_compile_features(main PRIVATE cxx_std_17)
add_dependencies(main ${catkin_EXPORTED_TARGETS})
target_link_libraries(main PUBLIC
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    common
    interface
    ezcfg::ezcfg

)


