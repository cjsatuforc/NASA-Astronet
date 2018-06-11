close all; clear; clc; rosshutdown;

setenv('ROS_MASTER_URI', 'http://192.168.0.107:11311');
rosinit('NodeName', '/matlab');


quad_list = {'/hbirdb/gazebo/set_model_state' '/hbirddg/gazebo/set_model_state'};
viconCallback_list = {@viconCallback_hbirdb @viconCallback_hbirddg};

% HBirdB Callback Variables
global x_1 y_1 z_1 qx_1 qy_1 qz_1 qw_1;

% HBirdDg Callback Variable
global x_2 y_2 z_2 qx_2 qy_2 qz_2 qw_2;

rospublisher(sprintf('/cov_ctrl_%d',1),'asctec_hl_comm/mav_ctrl')