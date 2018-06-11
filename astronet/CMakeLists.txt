cmake_minimum_required(VERSION 2.8.3)
project(aerolab)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_msgs geometry_msgs asctec_hl_comm)

catkin_package()

# specify location of header files
include_directories(include ${catkin_INCLUDE_DIRS})


# Make Test Nodes
# add_executable(publish nodes/test/publish.cpp)
# add_executable(command nodes/test/command.cpp)

# Make Useful Nodes
add_executable(publisher nodes/publisher.cpp)
add_executable(hbirdbState nodes/hbirdbState.cpp)
add_executable(hbirddgState nodes/hbirddgState.cpp)




# add target link libraries for each executable you make
# target_link_libraries(publish ${catkin_LIBRARIES})
# target_link_libraries(command ${catkin_LIBRARIES})
target_link_libraries(publisher ${catkin_LIBRARIES})
target_link_libraries(hbirdbState ${catkin_LIBRARIES})
target_link_libraries(hbirddgState ${catkin_LIBRARIES})
