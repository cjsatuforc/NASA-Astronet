close all
rosshutdown
clear
clc
%----------------------- ROS parameters ------------------------------%
setenv('ROS_MASTER_URI','http://dasc:11311/');
rosinit('NodeName','/Matlab');

%--- Global update variables
    
% Quaternion
global qx_1 qx_head qx_left qx_right;
global qy_1 qy_head qy_left qy_right;
global qz_1 qz_head qz_left qz_right;
global qw_1 qw_head qw_left qw_right;

% Position
global x_1 x_head x_left x_right;
global y_1 y_head y_left y_right;
global z_1 z_head z_left z_right;

quad_list = {'/vicon/hbirddg/hbirddg' '/vicon/William/William' '/vicon/Left_Arm/Left_Arm' '/vicon/Right_Arm/Right_Arm'};
viconCallback_list = {@viconCallback_quad @viconCallback_William @viconCallback_Left_Arm @viconCallback_Right_Arm};
statusCallback_list = {@statusCallback_1 @statusCallback_2 @statusCallback_3 @statusCallback_4};


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

vicon_sub(k+1)= rossubscriber(quad_list{k+1},'geometry_msgs/TransformStamped',viconCallback_list{k+1});
pause(2);
vicon_sub(k+2)= rossubscriber(quad_list{k+2},'geometry_msgs/TransformStamped',viconCallback_list{k+2});
pause(2);
vicon_sub(k+3)= rossubscriber(quad_list{k+3},'geometry_msgs/TransformStamped',viconCallback_list{k+3});
pause(2);

% Define loop rate
rate = rosrate(60);


%---------------------------- END ------------------------------------%



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
%Let's try to keep vehicle within 0.75 of me
dist_me=norm([x_head-x_i(kk),y_head-y_i(kk)])

if z_i(kk)<1.3
    v_z=0.5;
    
end
if z_i(kk)>1.3
    v_z=-0.5;
    
end

if dist_me<0.75

heading_vec=-[x_head-x_i(kk),y_head-y_i(kk)]./dist_me;
v_x=heading_vec(1);
v_y=heading_vec(2);
end
if dist_me>=0.75 && dist_me<1

heading_vec=[x_head-x_i(kk),y_head-y_i(kk)]./dist_me;
v_x=heading_vec(1);
v_y=heading_vec(2);
end
if dist_me>=1

heading_vec=[x_head-x_i(kk),y_head-y_i(kk)]./dist_me;
v_x=5*heading_vec(1);
v_y=5*heading_vec(2);
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


drawnow
waitfor(rate);
while z_left>1.3
    pos_rel_arm=[x_1-x_right;y_1-y_right;z_1-z_right];
    Orient_right=quat2eul([qw_right qx_right qy_right qz_right]);
    Rot_Mat_right=[cos(Orient_right(2))*cos(Orient_right(1)),(sin(Orient_right(3))*sin(Orient_right(2))*cos(Orient_right(1))-cos(Orient_right(3))*sin(Orient_right(1))),(cos(Orient_right(3))*sin(Orient_right(2))*cos(Orient_right(1))+sin(Orient_right(3))*sin(Orient_right(1))); ...
         cos(Orient_right(2))*sin(Orient_right(1)),(sin(Orient_right(3))*sin(Orient_right(2))*sin(Orient_right(1))+cos(Orient_right(3))*cos(Orient_right(1))),(cos(Orient_right(3))*sin(Orient_right(2))*sin(Orient_right(1))-sin(Orient_right(3))*cos(Orient_right(1))); ...
         -sin(Orient_right(2)),(sin(Orient_right(3))*cos(Orient_right(2))),(cos(Orient_right(3))*cos(Orient_right(2)))];
    pos_rel_arm=Rot_Mat_right\pos_rel_arm; %Now in the arm frame
    v_x=3;
    v_y=-3*pos_rel_arm(2);
    v_z=-3*pos_rel_arm(3);
    Back_to_room=Rot_Mat_right*[v_x;v_y;v_z];
    
    
    
    

Orient=quat2eul([qw_1 qx_1 qy_1 qz_1]);
Rot_Mat=[cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
         cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
         -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];
Control_body=Rot_Mat\Back_to_room;
ctrl_msg(kk).X=Control_body(1);
ctrl_msg(kk).Y=Control_body(2);
ctrl_msg(kk).Z=Control_body(3);   
send(ctrl_pub(kk),ctrl_msg(kk));
drawnow
waitfor(rate);
end
end

clear global qx qy qz qw x y z;
rosshutdown;
