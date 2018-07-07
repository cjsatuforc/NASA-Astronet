# BUILD DEPENDENCIES :
List of all the dependent repositories required to use the Astronet Package. All these are installed using ```install_astrobee.sh```.

1. The dependencies used are cloned from the following repositories
* [Vicon System](https://github.com/ethz-asl/vicon_bridge.git)
* [Catkin Glog](https://github.com/ethz-asl/glog_catkin.git)
* [Catkin Simple](https://github.com/catkin/catkin_simple.git)
* [Messages used for ASCTEC Hummingbird](https://github.com/ethz-asl/mav_comm.git)
* [ETH Zurich MSF Libraries](https://github.com/ethz-asl/ethzasl_msf.git)
* [Communication Framework for ASCTEC Hummingbirds](https://github.com/ethz-asl/asctec_mav_framework.git)

Make sure to have the gtest library installed else the MSF framework creates issues while building. The install script does it for you automatically. However, if you face issues, you can follow [this](https://www.eriksmistad.no/getting-started-with-google-test-on-ubuntu/) tutorial to install it.