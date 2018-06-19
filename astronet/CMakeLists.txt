cmake_minimum_required(VERSION 2.8.3)
project(aerolab)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_msgs geometry_msgs asctec_hl_comm)

catkin_package()

# specify location of header files
include_directories(include ${catkin_INCLUDE_DIRS})


# TEST NODES
# add_executable(publish nodes/test/publish.cpp)
# add_executable(command nodes/test/command.cpp)

# target_link_libraries(publish ${catkin_LIBRARIES})
# target_link_libraries(command ${catkin_LIBRARIES})


# PUBLISHER/SUBSCRIBER MAIN NODES
add_executable(hbirdbPub nodes/hbirdbPub.cpp)
add_executable(hbirddgPub nodes/hbirddgPub.cpp)
add_executable(hbirdbState nodes/hbirdbState.cpp)
add_executable(hbirddgState nodes/hbirddgState.cpp)
add_executable(viconviz nodes/viconViz.cpp)

target_link_libraries(hbirdbPub ${catkin_LIBRARIES})
target_link_libraries(hbirddgPub ${catkin_LIBRARIES})
target_link_libraries(hbirdbState ${catkin_LIBRARIES})
target_link_libraries(hbirddgState ${catkin_LIBRARIES})
target_link_libraries(viconviz ${catkin_LIBRARIES})
