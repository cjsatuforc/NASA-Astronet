# export paths
export SOURCE_PATH=$HOME/Documents/Astrobee/Freeflyer
export ANDROID_PATH="${SOURCE_PATH}_android"
export BUILD_PATH=$HOME/Documents/Astrobee/Freeflyer_build
export INSTALL_PATH=$HOME/Documents/Astrobee/Freeflyer_install

# clone repos
git clone https://github.com/sahibdhanjal/NASA-Astronet.git $SOURCE_PATH
git clone https://github.com/nasa/astrobee_android.git $ANDROID_PATH

# install dependencies
pushd $SOURCE_PATH
cd scripts/setup
./add_ros_repository.sh
cd debians
./build_install_debians.sh
cd ../
./install_desktop_16_04_packages.sh
sudo rosdep init
rosdep update
popd

# Configure Build
pushd $SOURCE_PATH
./scripts/configure.sh -l -F -D -p $INSTALL_PATH -b $BUILD_PATH
popd

# Build the Code
pushd $BUILD_PATH
make -j6
popd

# Add to bashrc file
echo "source ~/Documents/Astrobee/Freeflyer_build/devel/setup.bash" >> ~/.bashrc


# Make Astronet Simulator Workspace
mkdir -p ~/Documents/Astronet/src
cd ~/Documents/Astronet/src
catkin_init_workspace
cd ..
catkin_make

# Build base dependency libraries
cd src/
git clone https://github.com/ethz-asl/vicon_bridge.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/mav_comm.git
cd mav_comm
sudo mv -r * ../
cd ../..
catkin_make

# Build framework libraries
cd src/
git clone https://github.com/ethz-asl/asctec_mav_framework.git
git clone https://github.com/ethz-asl/ethzasl_msf.git
cd ../..
catkin_make