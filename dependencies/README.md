#Build Instructions:

<!-- Make the Workspace as follows -->
mkdir -p ~/Documents/catkin_ws/src
cd ~/Documents/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make

<!-- Clone Base Repositories in src folder -->
cd src/
git clone https://github.com/ethz-asl/vicon_bridge.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/catkin/catkin_simple.git

<!-- Clone the message repo and move it to the main src folder -->
git clone https://github.com/ethz-asl/mav_comm.git
cd mav_comm
sudo mv -r * ../

<!-- Make the Base Target Libraries -->
cd ../..
catkin_make

<!-- Once made, add the main framework libraries -->
cd src/
git clone https://github.com/ethz-asl/asctec_mav_framework.git
git clone https://github.com/ethz-asl/ethzasl_msf.git

<!-- goto the CMakeLists.txt of msf_timing and msf_core
	and add the path to the gtest library if it creates an error
	while compiling -->
## Add the path to gtest:
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
if(NOT GTEST_LIBRARY)
   message("not found")
endif()

<!-- finally make again -->
catkin_make