rosshutdown;
clear;
clc;
close all;


setenv('ROS_MASTER_URI','http://dasc:11311/');
rosinit;


global ptcloud

ptcloud = rosmessage('sensor_msgs/LaserScan');


sub = rossubscriber('/leddar/scan',@lasercallback);
pause(2);


rate = rosrate(50);

while 1
    x=length(ptcloud.Ranges);
    scatter((1:x)',ptcloud.Ranges)
    waitfor(rate);
end
