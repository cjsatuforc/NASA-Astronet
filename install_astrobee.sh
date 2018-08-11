###########################################
# Compile and Build Astrobee Base Libraries
###########################################

# export paths
export SOURCE_PATH=$HOME/Documents/Astrobee/Astrobee
export ANDROID_PATH="${SOURCE_PATH}_android"
export BUILD_PATH=$HOME/Documents/Astrobee/Astrobee_build
export INSTALL_PATH=$HOME/Documents/Astrobee/Astrobee_install
export ASTRONET_PATH=$HOME/Documents/Astrobee/Astronet
export DEPEND_PATH=$HOME/Documents/Astrobee/Dependencies

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

###########################################
# Build Dependencies
###########################################
# install gtest 
sudo apt-get install libgtest-dev
sudo apt-get install cmake
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib

# build dependencies
mkdir -p $DEPEND_PATH/src
cd $DEPEND_PATH/src
git clone https://github.com/ethz-asl/vicon_bridge.git
cd $DEPEND_PATH
catkin_make
sudo mv -v $SOURCE_PATH/dependencies/* $DEPEND_PATH/src/
catkin_make
# Perform once again to confirm build
catkin_make


###########################################
# Build Astronet
###########################################
mkdir -p $ASTRONET_PATH/src
sudo mv -v $SOURCE_PATH/aerolab/* $ASTRONET_PATH/src/
cd $ASTRONET_PATH
catkin_make


###########################################
# Clean/Remove Temp Directories
###########################################
rm -rf $SOURCE_PATH/aerolab $SOURCE_PATH/dependencies


###########################################
# export paths and source from setup files
###########################################
echo 'export SOURCE_PATH=$HOME/Documents/Astrobee/Astrobee' >> ~/.bashrc
echo 'export ANDROID_PATH="${SOURCE_PATH}_android"' >> ~/.bashrc
echo 'export BUILD_PATH=$HOME/Documents/Astrobee/Astrobee_build' >> ~/.bashrc
echo 'export INSTALL_PATH=$HOME/Documents/Astrobee/Astrobee_install' >> ~/.bashrc
echo 'export ASTRONET_PATH=$HOME/Documents/Astrobee/Astronet' >> ~/.bashrc
echo 'export DEPEND_PATH=$HOME/Documents/Astrobee/Dependencies' >> ~/.bashrc

echo 'source $BUILD_PATH/devel/setup.bash' >> ~/.bashrc
echo 'source $ASTRONET_PATH/devel/setup.bash' >> ~/.bashrc
echo 'source $DEPEND_PATH/devel/setup.bash' >> ~/.bashrc