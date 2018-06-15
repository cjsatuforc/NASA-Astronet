# BUILD INSTRUCTIONS :

1. Make the Workspace as follows

```bash
mkdir -p ~/Documents/catkin_ws/src
cd ~/Documents/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

2. Clone Base Repositories in src folder
```bash
cd src/
git clone https://github.com/ethz-asl/vicon_bridge.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/catkin/catkin_simple.git
```

3. Clone the message repo and move it to the main src folder
```bash
git clone https://github.com/ethz-asl/mav_comm.git
cd mav_comm
sudo mv -r * ../
```

4. Make the Base Target Libraries
```bash
cd ../..
catkin_make
```

5. Once made, add the main framework libraries
```bash
cd src/
git clone https://github.com/ethz-asl/asctec_mav_framework.git
git clone https://github.com/ethz-asl/ethzasl_msf.git
```

6. Now goto the CMakeLists.txt of msf_timing and msf_core and add the path to the gtest library if it creates an error while compiling
```bash
# Add the path to gtest:
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
if(NOT GTEST_LIBRARY)
   message("not found")
endif()
```

7. Finally make again
```bash
catkin_make
```
