close all
rosshutdown
clear
clc
%----------------------- ROS parameters ------------------------------%
setenv('ROS_MASTER_URI','http://dasc:11311/');
rosinit('NodeName','/Matlab');

%--- Global update variables
    
% Quaternion
global qx_1 qx_2 qx_3 qx_4 qx_5 qx_6;
global qy_1 qy_2 qy_3 qy_4 qy_5 qy_6;
global qz_1 qz_2 qz_3 qz_4 qz_5 qz_6;
global qw_1 qw_2 qw_3 qw_4 qw_5 qw_6;

% Position
global x_1 x_2 x_3 x_4 x_5 x_6;
global y_1 y_2 y_3 y_4 y_5 y_6;
global z_1 z_2 z_3 z_4 z_5 z_6;

% Battery voltage
global volt_1 volt_2 volt_3 volt_4 volt_5 volt_6;

quad_list = {'/vicon/hbirddg/hbirddg' '/vicon/hbirdb/hbirdb' '/vicon/hbirdw/hbirdw' '/vicon/hbirdg/hbirdg' '/vicon/hbirdYE/hbirdYE' '/vicon/hbirdgr/hbirdgr'};
viconCallback_list = {@viconCallback_1 @viconCallback_2 @viconCallback_3 @viconCallback_4 @viconCallback_5 @viconCallback_6};
statusCallback_list = {@statusCallback_1 @statusCallback_2 @statusCallback_3 @statusCallback_4 @statusCallback_5 @statusCallback_6};


%Number of quadrotors
k=1; 



for i=1:k
    vicon_sub(i) = rossubscriber(quad_list{i},'geometry_msgs/TransformStamped',viconCallback_list{i});
    pause(2);

    mav_state_sub(i) = rossubscriber(strcat(strcat('/quad',num2str(i)),'/fcu/status'),'asctec_hl_comm/mav_status',statusCallback_list{i});
    pause(2)
    
    ctrl_pub(i) = rospublisher(sprintf('/cov_ctrl_%d',i),'asctec_hl_comm/mav_ctrl');
    ctrl_msg(i) = rosmessage(ctrl_pub(i));
    pause(2);
end

% Define loop rate
rate = rosrate(60);


%---------------------------- END ------------------------------------%

tic
t=1;
while 1==1;

    
%Copy global varaibles from vicon callback to local variables. No reason...
x_i(1)=x_1;
y_i(1)=y_1;
z_i(1)=z_1;
%Add additional lines here for more agents
%For reference, your orientation is qw_1 qx_1 etc. And quad2eul is a useful
%function


for kk=1:k

ctrl_msg(kk).Type = 2;
%Cartesian X,Y,Z velocities and Yaw Rate
ctrl_msg(kk).X=0;
ctrl_msg(kk).Y=0;
ctrl_msg(kk).Z=0;
ctrl_msg(kk).Yaw=0;
%Just an example

%Now example controller, bang bang


if z_i(kk)<-50
    v_z=0.2;
    
end
if z_i(kk)>-50
     v_z=-0.2;
    
end

if x_i(kk)<100
    v_x=0.2;
    
end
if x_i(kk)>100
     v_x=-0.2;
    
end

if y_i(kk)<100
    v_y=0.2;
    
end
if y_i(kk)>100
     v_y=-0.2;
    
end


Orient=quat2eul([qw_1 qx_1 qy_1 qz_1]);
Rot_Mat=[cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
         cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
         -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];

Control_body=Rot_Mat\[v_x;v_y;v_z];
ctrl_msg(kk).X=Control_body(1);
ctrl_msg(kk).Y=Control_body(2);
ctrl_msg(kk).Z=Control_body(3);
send(ctrl_pub(kk),ctrl_msg(kk));  

end
fprintf('Running\n');
pos=[x_1 y_1 z_1]
ori=quat2eul([qw_1 qx_1 qy_1 qz_1])

time(t)=toc;
x_i(t)=x_1;
y_i(t)=y_1;
z_i(t)=z_1;
if t>1
vx_i(t)=(2.55/200)*(x_i(t)-x_i(t-1))/(time(t)-time(t-1));
vy_i(t)=(2.55/200)*(y_i(t)-y_i(t-1))/(time(t)-time(t-1));
vz_i(t)=(2.3/100)*(z_i(t)-z_i(t-1))/(time(t)-time(t-1));
end

t=t+1;
drawnow
waitfor(rate);

end

clear global qx qy qz qw x y z;
rosshutdown;
