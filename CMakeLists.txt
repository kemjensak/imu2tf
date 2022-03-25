cmake_minimum_required(VERSION 3.0.2)
project(imu2tf)

add_compile_options(-O -Wall -std=c++14 -fpermissive)


find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    tf2_ros
    sensor_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    tf2_ros
    sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(imu2link_tf
    src/imu2link_tf.cpp)

add_dependencies(imu2link_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(imu2link_tf
    ${catkin_LIBRARIES}
)


add_executable(enu2bodyfixed
src/enu2bodyfixed.cpp)

add_dependencies(enu2bodyfixed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(enu2bodyfixed
    ${catkin_LIBRARIES}
)

