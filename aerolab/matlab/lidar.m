rosshutdown;
clear;
clc;

setenv('ROS_MASTER_URI','http://dasc:11311/');
rosinit;


global ptcloud

ptcloud = rosmessage('sensor_msgs/PointCloud2');


sub = rossubscriber('/velodyne_points',@ptcallback);
pause(2);


rate = rosrate(50);

while 1
    xyz = readXYZ(ptcloud);
%     a=1:length(xyz);
%     distance = sqrt(xyz(:,1).^2 + xyz(:,2).^2 + xyz(:,3).^2);
    scatter3(xyz(:,1), xyz(:,2), xyz(:,3), '.');
%     plot(xyz(:,1), xyz(:,2),'.');
%     scatter(a,distance,'.')
    waitfor(rate);
end

