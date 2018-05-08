# Astrobee Robot Software v1

## Release 0.3.2

The previous hotfix updated the debians, but did not update the version numbers of which
debians to install. This corrects that.

## Release 0.3.1

This hotifx fixes the build with ROS' new opencv release, which moved the opencv libraries
from /opt/ros/kinetic/lib to /opt/ros/kinetic/lib/PLATFORM, breaking our RPATHs.

## Release 0.3.0

This release addresses several bugs identified by Guest Scientists, simulator improvements as well as many internal changes.
The only API change is in the EkfState (removed augmented state).

### Simulator
  - Stability improvements
  - Better performance by optimized collision checking and new method for feature simulation
  - More detailed CAD models and textures

### Mobility
  - New mapper node builds an Octomap from the HazCam data
  - Collision checking performed at runtime agains the Octomap
  - Docking and undocking complete

### Limitations
  - Angular velocities/acceleration in GDS not consistent with FSW (will be fixed in GDS)
  - Keep in and keep out zones are not checked (during validation or execution)
  - Simulated docking in a beta stage and should not be used

## Release 0.2.0

### Simulator
  - Perching Arm motion (and dynamics) functional
  - PerchCam and HazCam depth sensors enabled
  - Improved performance

### Guest Science
  - Guest Science Manager implementation available on Android
  - Guest Science library for communication between the manager and guest apps
  - Executive can command Guest Science apps

### Architecture
  - GN&C decomposed into three separate ROS nodes for easier customization / extension:
    - EKF
    - Control
    - Force Allocation Module
  - Management of FlightModes handled on all levels (Exec, Mobility,
    GN&C and Propulsion)

### Localization
  - Feature map much smaller
  - Faster sparse mapping (3Hz on 2 cores only versus 2Hz on 4 cores)
  - Visualization tool for localization features
  - Calibration of depth cameras relative to IMU

### Mobility
  - Fixed 6DoF face-forward bug with the trapezoidal path planner
  - Asynchronous, state-based mobility and control pipeline with improved debugging output
  - Multiple speed gains now supported by FAM, with a controlled ramp up/down
  - Robot now starts in "off" flight mode by default
  - Asychronous, state-based procedures for
    - Arm control
    - Docking and undocking
    - Perching and unperching

### Executive
  - Changed to new motion system
  - Flight mode propagation

### Hardware drivers
  - Picoflexx driver produces depth images in addition to point clouds
  - Picoflexx internal core (Royale) driver bumped to v3.9.0 LTS
  - PMC actuator now determines its state (ramping up/down, ready) based on telemetry feedback

## Release 0.1.2

### Simulator
  - Use the same flight software stack that is run on the platform (but the HW
    drivers)
  - Dynamics of the Astrobee using Gazebo at 1KHz
  - One ISS module environment from Gazebo
  - GNC control running at 62.5Hz (same as real platform)
  - uses GNC Simulink blower propulsion module
  - EKF inputs:
    - IMU model (no noise in this release)
    - Sparse mapping features (sampled from point cloud)
    - Visual features from synthetic images for optical flow
    - Camera models (from Gazebo with radial distortion)
  - Flashlight and laser representation
  - Can run with Gazebo 7 GUI and RViz or headless
  - Supports muti-Astrobee simulation (no communication between them)
  - Collision between Astrobee and ISS walls simulated

#### Limitations
  - No AR target tracking
  - No Handrail detection
  - Depthcam point cloud disabled (efficiency)
  - Camera raytracing runs at 2Hz
  - Conservative limit on angular velocity
  - Arm simulation disabled
  - No noise in the system


### Guest Science
  - JAVA API generated from XP-JSON command dictionary
  - Commands can be send to the Executive using the API and provided ROS Java
    framework

#### Limitations
  - Android framework to support ROS Java in development
  - No guest science manager (life cycle of Guest Science apps)

### Localization
  - Localize without external infrastructure (beacons, etc.)
  - EKF works with following inputs:
    - Sparse mapping with BRISK (regular nav), ~2Hz
    - Optical flow ~15Hz
    - AR targets (docking)
    - Handrail detection (perching)
  - Produce 62.5Hz output
  - Localization manager allows switching safely between localization modes
  - Tools to calibrate intrinsics and extrinsics (IMU to Camera)
  - Tools to build maps from monocular vision (SURF + BRISK)
  - Tools to test localization performance from rosbag
  - Custom visualizer (gViz) for EFK inspection

#### Limitations
  - Sparse mapping runs only at 2Hz (vision processing + feature matching)
  - High distortion limits the usable features
  - Handrail localization needs improvements
  - No extrinsics calibration between depthcam and IMU
  - Better tuning for noise model and "lost" threshold required
  - No incremental map building
  - No tiling of large maps
  - No perched localization (low power localization when perched)

### Mobility
  - Provides 4 different motion types:
    - IDLE (drifting)
    - STOP (zeros velocity, hold position if not externally moved)
    - MOVE (moves to given end pose)
    - EXECUTE (follows a time/space trajectory)
  - Provides 2 motion planners:
    - trapezoidal (default)
    - QP planner
  - Move primitive are position based
  - Validation of trajectories: stationary end-point, hard limits (velocity and
    acceleration) and control frequency
  - Velocity control is not allowed

#### Limitations
  - On-board validation of trajectories against keep-in and keep-out not enabled
  - Planner type cannot be changed from ground (DDS)
  - Hazard detection not implemented

### Executive
  - Arbitrates commands regarding operating mode and current state
  - Sequencer supports plan execution
  - Supports essential commands (see list of currently supported commands)
    - Plan
    - Motion
    - Camera management
   - System management

#### Limitations
  - Full command dictionary is not implemented
  - Guest Science management not supported yet

### Fault Management
  - Faults are managed in a spreadsheet, converted to a fault table read by the
    system
  - Heart beat monitor for most nodes
  - System monitor framework can trigger responses to published faults

#### Limitations
  - Very few faults are published

### Framework
  - Leverages ROS framework for on-board framework
  - All software stack works on Ubuntu 16.04 on Intel and ArmHF (target
    platform)
  - Dependencies packaged as Debian
  - Software updates delivered as Debian
  - Unify launch file system supports multiple scenarios, on-robot, simulator or
    processor in the loop configuration
  - Leverages rViz and Gazebo 7 integration
  - Improved HTC Vive tracker, calibration procedure and integration for ground
    truth

#### Limitations
  - Documentation is incomplete
