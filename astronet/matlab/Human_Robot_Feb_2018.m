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
tic

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

%Set Up HRI Parameters

%Tunable Parameters
Beta=1; %Human Rationality Index
K=5; %Number of Goals
R=5; %Terminal Reward
c=1; %Running Cost
R_max=10; %Robot max speed
H_max=2*sqrt(8)/5; %Human max speed
cap_tol=2.5; %Capture radius
Capture_List=[];
att_rad=10; %attractive radius
gamma=0.9; %discount factor
%Set up Environment, from 1 up to 100 units.
x_max=50;
y_max=50;
[x_mesh,y_mesh]=meshgrid(1:x_max,1:y_max);
t=1;
P_a_sg=zeros(x_max,y_max,K);
x_orig=-1.2;
y_orig=-2.6;
%Place goals at random locations
goal(1,1)=(1.2457-x_orig)*10;
goal(1,2)=(-2.4217-y_orig)*10;

goal(2,1)=(1.2269-x_orig)*10;
goal(2,2)=(-0.6072-y_orig)*10;

goal(3,1)=(0.8477-x_orig)*10;
goal(3,2)=(1.7150-y_orig)*10;

goal(4,1)=(0.2662-x_orig)*10;
goal(4,2)=(1.3679-y_orig)*10;

goal(5,1)=(-0.3188-x_orig)*10;
goal(5,2)=(1.6087-y_orig)*10;

%Human Starting Location
Human(t,1)=(x_head-x_orig)*10;
Human(t,2)=(y_head-y_orig)*10;

%Robot Starting Location
Robot(t,1)=(x_1-x_orig)*10;
Robot(t,2)=(y_1-y_orig)*10;






while K>0
%Initialize prior belief of next human goal
for k=1:K
dist_h_g(k)=1/norm((Human(t,:)-goal(k,:)));
end
P_g_s=dist_h_g./sum(dist_h_g(:));
P_g_s=transpose(P_g_s);

%Compute every possible path through the network and it's path length
    paths=perms(1:K);
    paths(:,K+1)=0;
    paths(:,K+2)=sqrt((goal(paths(:,1),1)-Human(t,1)).^2+(goal(paths(:,1),2)-Human(t,2)).^2);
    for j=1:K-1
    paths(:,K+1)=paths(:,K+1)+sqrt((goal(paths(:,j),1)-goal(paths(:,j+1),1)).^2+(goal(paths(:,j),2)-goal(paths(:,j+1),2)).^2);
    %paths K+2j is human_collab cost and K+2j+1 is robot collab cost
    %j=1 corresponds to human with first task and robot with all others, 
    if j<K-1
    paths(:,K+2*(j+1))=paths(:,K+2*j)+sqrt((goal(paths(:,j+1),1)-goal(paths(:,j),1)).^2+(goal(paths(:,j+1),2)-goal(paths(:,j),2)).^2);
    end
    end
    for j=1:K-1
    paths(:,K+2*j+1)=paths(:,K+2*j-1)-sqrt((goal(paths(:,j+1),1)-goal(paths(:,j),1)).^2+(goal(paths(:,j+1),2)-goal(paths(:,j),2)).^2);
    end
    for j=1:K-1
    paths(:,K+2*j+1)=paths(:,K+2*j+1)+sqrt((goal(paths(:,j+1),1)-Robot(t,1)).^2+(goal(paths(:,j+1),2)-Robot(t,2)).^2);
    end

  
  for j=1:K-1
  possible_collab_lengths(:,j)=max(paths(:,K+2*j),paths(:,K+2*j+1));
  end
  %change this to have K possible Robot goals instead and select them
  %online later
  if K>1
  Robot_Goals=ones(K,K-1);
  for k=1:K
  possible_collab_temp=possible_collab_lengths((k-1)*factorial(K-1)+1:k*factorial(K-1),:);
  [min_val,idx]=min(possible_collab_temp(:));
  
  idx=idx+ceil(idx/factorial(K-1))*(k-1)*factorial(K-1)+(ceil(idx/factorial(K-1))-1)*(K-k)*factorial(K-1);
  
  [Sequence,Collab_point]=ind2sub(size(possible_collab_lengths),idx);
  Robot_Goals(k,1:length(paths(Sequence,Collab_point+1:K)))=paths(Sequence,Collab_point+1:K)
  end

  
  else
  Robot_Goals=1;    
  end

while(1)
ctrl_msg(1).Type = 2;
%Cartesian X,Y,Z velocities and Yaw Rate
ctrl_msg(1).X=0;
ctrl_msg(1).Y=0;
ctrl_msg(1).Z=0;
ctrl_msg(1).Yaw=0;
%Just an example
drawnow
waitfor(rate);

    Human_feas_zone=zeros(y_max,x_max);
    for x=1:x_max
        for y=1:y_max
       if norm([x-Human(t,1),y-Human(t,2)])<=H_max
          Human_feas_zone(y,x)=1; 
       end
        end
    end
    %Cycle through each goal
    for k=1:K
    P_a_sg(:,:,k)=exp(Beta.*(gamma.^(sqrt((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2))*R-c*((gamma-gamma.^(sqrt((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2)))/(1-gamma))));
    P_a_sg(:,:,k)=squeeze(P_a_sg(:,:,k)).*Human_feas_zone;
    P_a_sg(:,:,k)=P_a_sg(:,:,k)/sum(sum(P_a_sg(:,:,k)));
    end
    %Human moves 1 unit towards goal
    Human(t+1,1)=(x_head-x_orig)*10;
    Human(t+1,2)=(y_head-y_orig)*10;
    %Now Update Distribution
    P_g_s=squeeze(P_a_sg(max(1,min(int64(Human(t+1,2)),y_max)),max(1,min(int64(Human(t+1,1)),x_max)),:)).*P_g_s;
    P_g_s=P_g_s./sum(P_g_s)
    
    Robot_Goal_P=zeros(K,1);
    for k=1:K
     %Robot_Goal_P(opt_path(k,K))=Robot_Goal_P(opt_path(k,K))+P_g_s(k);
     Robot_Goal_P(Robot_Goals(k,1))=Robot_Goal_P(Robot_Goals(k,1))+P_g_s(K+1-k);
    end
    Robot_feas_zone=inf*ones(y_max,x_max);
    
    for x=1:x_max
        for y=1:y_max
       if norm([x-Robot(t,1),y-Robot(t,2)])<=R_max
          Robot_feas_zone(y,x)=1; 
       end
        end
    end
    Robot_opt_map=zeros(y_max,x_max);
    normalizer=0;
    for k=1:K
    Robot_opt_map=Robot_opt_map+(max(0,(-log(sqrt(((Robot(t,1)-goal(k,1)).^2+(Robot(t,2)-goal(k,2)).^2))./att_rad)))+Robot_Goal_P(k)).*sqrt((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2);
    normalizer=normalizer+((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2);
    end
    %Robot_opt_map=Robot_opt_map./normalizer;
    Robot_opt_map=Robot_opt_map.*Robot_feas_zone;
    [garbage,rob_dex]=min(Robot_opt_map(:));
    [y_rob_dest,x_rob_dest]=ind2sub(size(Robot_opt_map),rob_dex);
    %Propagate Robot State
   
    SEND_DEST_X=(x_rob_dest/10)+x_orig;
    SEND_DEST_Y=(y_rob_dest/10)+y_orig;
    
    if z_1(1)<1.3
    v_z=0.5;
    end
    if z_1(1)>1.3
    v_z=-0.5;
    end
    
    heading_vec=[SEND_DEST_X-x_1,SEND_DEST_Y-y_1]./norm([SEND_DEST_X-x_1,SEND_DEST_Y-y_1]);
    v_x=5*heading_vec(1);
    v_y=5*heading_vec(2);
    Orient=quat2eul([qw_1 qx_1 qy_1 qz_1]);
    Rot_Mat=[cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
         cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
         -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];

    Control_body=Rot_Mat\[v_x;v_y;v_z];
    ctrl_msg(1).X=Control_body(1);
    ctrl_msg(1).Y=Control_body(2);
    ctrl_msg(1).Z=Control_body(3);
    send(ctrl_pub(1),ctrl_msg(1));  
    
    
    Robot(t+1,1)=(x_1-x_orig)*10;
    Robot(t+1,2)=(y_1-y_orig)*10;
    z_i(t+1)=z_1;
    P_save(1:length(P_g_s),t)=P_g_s;
    Time_save(t)=toc;
t=t+1;

%Check capture of target and break infinite loop
if min(sqrt((Human(t,1)-goal(:,1)).^2+(Human(t,2)-goal(:,2)).^2))<cap_tol || min(sqrt((Robot(t,1)-goal(:,1)).^2+(Robot(t,2)-goal(:,2)).^2))<cap_tol

    if min(sqrt((Human(t,1)-goal(:,1)).^2+(Human(t,2)-goal(:,2)).^2))<cap_tol
         
        [garbage,dex]=min(sqrt((Human(t,1)-goal(:,1)).^2+(Human(t,2)-goal(:,2)).^2));
        %[x position, y position, agent 1, time]
        Capture_List=[Capture_List;goal(dex,:),1,t];
        goal(dex,:)=[];
        K=K-1;
       
        
    end
    
    if min(sqrt((Robot(t,1)-goal(:,1)).^2+(Robot(t,2)-goal(:,2)).^2))<cap_tol
        
        [garbage,dex]=min(sqrt((Robot(t,1)-goal(:,1)).^2+(Robot(t,2)-goal(:,2)).^2));
        %[x position, y position, agent 2, time]
        Capture_List=[Capture_List;goal(dex,:),2,t];
        goal(dex,:)=[];
        K=K-1;
    end
clear dist_h_g P_g_s paths opt_dex opt_path P_a_sg possible_collab_lengths possible_collab_temp Robot_Goals
break
end
    
end

Final_Time=Capture_List(end,end)
%End of outer loop
end

clear global qx qy qz qw x y z;
rosshutdown;
