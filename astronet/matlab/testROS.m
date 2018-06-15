close all; clear; clc; rosshutdown;

setenv('ROS_MASTER_URI', 'http://localhost:11311');
rosinit('NodeName', '/matlab');

quad_list = {'/hbirdbState' '/hbirddgState' '/vicon/William/William' '/vicon/Left_Arm/Left_Arm' '/vicon/Right_Arm/Right_Arm'};
viconCallback_list = {@viconCallback_hbirdb @viconCallback_hbirddg @viconCallback_William @viconCallback_Left_Arm @viconCallback_Right_Arm};

% Quaternion
global qx_1 qx_2 qx_head qx_left qx_right;
global qy_1 qy_2 qy_head qy_left qy_right;
global qz_1 qz_2 qz_head qz_left qz_right;
global qw_1 qw_2 qw_head qw_left qw_right;

% Position
global x_1  x_2 x_head x_left x_right;
global y_1  y_2 y_head y_left y_right;
global z_1  z_2 z_head z_left z_right;


% subscribe to gazebo states of the models of both quads
vicon_sub(1) = rossubscriber(quad_list{1},'geometry_msgs/Transform',viconCallback_list{1});
vicon_sub(2) = rossubscriber(quad_list{2},'geometry_msgs/Transform',viconCallback_list{2});

% subscribe to the head, left_arm, right_arm positions published by Vicon
vicon_sub(3)= rossubscriber(quad_list{3},'geometry_msgs/TransformStamped',viconCallback_list{3}); 
vicon_sub(4)= rossubscriber(quad_list{4},'geometry_msgs/TransformStamped',viconCallback_list{4});
vicon_sub(5)= rossubscriber(quad_list{5},'geometry_msgs/TransformStamped',viconCallback_list{5});