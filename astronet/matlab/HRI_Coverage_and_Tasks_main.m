%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Head - 
% min - (-0.095, -0.972, 0)
% max - (0.55, 0.19, 1.865)
% avg - (0.095, -0.174, 1.843)
% 
% Left Arm - 
% min - (-0.365, -1.036, 0)
% max - (0.785, 0.284, 2.135)
% avg - (0.275, -0.275, 1.713)
% 
% Right Arm -
% min - (-0.546, -0.79, 0)
% max - (0.75, 0.4, 2.134)
% avg - (-0.06, -0.22, 1.46)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rosshutdown, clear, clc;
close all
tic

% Global variables
% Quaternion
global qx_1 qx_2 qx_head qx_left qx_right;
global qy_1 qy_2 qy_head qy_left qy_right;
global qz_1 qz_2 qz_head qz_left qz_right;
global qw_1 qw_2 qw_head qw_left qw_right;

% Position
global x_1  x_2 x_head x_left x_right;
global y_1  y_2 y_head y_left y_right;
global z_1  z_2 z_head z_left z_right;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------- ROS parameters - Sahib -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setenv('ROS_MASTER_URI', 'http://localhost:11311');
rosinit('NodeName', '/matlab');

quad_list = {'/hbirdbState' '/hbirddgState' '/vicon/William/William' '/vicon/Left_Arm/Left_Arm' '/vicon/Right_Arm/Right_Arm'};
viconCallback_list = {@viconCallback_hbirdb @viconCallback_hbirddg @viconCallback_William @viconCallback_Left_Arm @viconCallback_Right_Arm};

quad_num = 2;
tic

% subscribe to gazebo states of the models of both quads
vicon_sub(1) = rossubscriber(quad_list{1},'geometry_msgs/Transform',viconCallback_list{1});
vicon_sub(2) = rossubscriber(quad_list{2},'geometry_msgs/Transform',viconCallback_list{2});

% subscribe to the head, left_arm, right_arm positions published by Vicon
vicon_sub(quad_num+1)= rossubscriber(quad_list{quad_num+1},'geometry_msgs/TransformStamped',viconCallback_list{quad_num+1}); 
vicon_sub(quad_num+2)= rossubscriber(quad_list{quad_num+2},'geometry_msgs/TransformStamped',viconCallback_list{quad_num+2});
vicon_sub(quad_num+3)= rossubscriber(quad_list{quad_num+3},'geometry_msgs/TransformStamped',viconCallback_list{quad_num+3});

% Define loop rate
rate = rosrate(60);

% publish control messages to both quads
for i=1:quad_num
    ctrl_pub(i) = rospublisher(sprintf('/cov_ctrl_%d',i),'asctec_hl_comm/mav_ctrl');
    ctrl_msg(i) = rosmessage(ctrl_pub(i));
    pause(1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------- SETUP HRI PARAMETERS -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set Up HRI Parameters

%Tunable Parameters
Beta=1; %Human Rationality Index
K=5; %Number of Goals
R=10; %Terminal Reward
c=1; %Running Cost
R_max=10; %Robot max speed
H_max=sqrt(2); %Human max speed
dttt=.1;
cap_tol=2.5; %Capture radius
Capture_List=[];
att_rad=10; %attractive radius
gamma=0.55; %discount factor

%Decently okay parameters are gamma=.55 R=8 c=1


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------------- SETUP ENVIRONMENT -------------------------%
% units in dm implies that (x-max, y-max): (5m, 5m) and origin at 
% (-1.2, -2.6). and 5 goal positions given randomly. Human position is
% taken wrt to origin, and similarly robots wrt to origin.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Set up Environment, from 1 up to 100 units.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------- SIM ENVIRONMENT PARAMETERS ---------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_min = -10; x_max = 52;
y_min = -8; y_max = 8;
z_min = 40; z_max = 57;

% Set origin for the simulator
% to be subtracted from simulator data
x_orig = (x_min)/10;
y_orig = (y_min)/10;
z_orig = (z_min)/10;

% Set origin for the actual physical workspace
% to be subtracted from vicon data
x_orig_ws = -1.2;
y_orig_ws = -2.6;
z_orig_ws = 0;

% arm height threshold
arm_thresh = 1.5;

% defining velocity scale
scale = 0.2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------------- SIM ENVIRONMENT DIMS ------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[x_mesh,y_mesh] = meshgrid(1:x_max-x_min+1,1:y_max-y_min+1);
t=1;

%Place goals at random locations
goal(1,1)=((x_max - x_min)*rand() + 1)*10;
goal(1,2)=((y_max - y_min)*rand() + 1)*10;

goal(2,1)=((x_max - x_min)*rand() + 1)*10;
goal(2,2)=((y_max - y_min)*rand() + 1)*10;


goal(2,1)=((x_max - x_min)*rand() + 1)*10;
goal(2,2)=((y_max - y_min)*rand() + 1)*10;

goal(3,1)=((x_max - x_min)*rand() + 1)*10;
goal(3,2)=((y_max - y_min)*rand() + 1)*10;

goal(4,1)=((x_max - x_min)*rand() + 1)*10;
goal(4,2)=((y_max - y_min)*rand() + 1)*10;

goal(5,1)=((x_max - x_min)*rand() + 1)*10;
goal(5,2)=((y_max - y_min)*rand() + 1)*10;

%Human Starting Location
Human(t,1)=(x_head-x_orig_ws)*10;
Human(t,2)=(y_head-y_orig_ws)*10;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------------- NORMALIZE POSITIONS -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_1 = x_1 - x_orig;
y_1 = y_1 - y_orig;
z_1 = z_1 - z_orig;
x_2 = x_2 - x_orig;
y_2 = y_2 - y_orig;
z_2 = z_2 - z_orig;

%Robot Starting Location
Robot(t,1) = x_1*10;
Robot(t,2) = y_1*10;
z_one(1) = z_1*10;
Time_save(1)=0;
D_z=0;
I_z=0;
P_z=0;
Capture_Flag=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------  SETUP COVERAGE PARAMETERS -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set up Coverage Parameters
c_star=1;
K_u=.004; K_v=.004; K_w=.001; K_r=4e-3; K_s=4e-3; K_vel=1;
U_max=.2; r_sat=0.02; s_sat=0.02;
r_i=3;
r_part=2;
%Size of Sensor Footprint
Ri=10;
alphaa=(1/2)*90*pi/180;
update_count=0;
orient_tol=.01;
%New on 5/19/16
Ep_star=.05; %Code Test Value

 
%set up coordinates
%NOminal are .5 and .1....Works well! 
dx=1; dy=1; dz=1; droll=.05; dpitch=dx/Ri; dyaw=dx/Ri;
%set(gcf,'visible','off')
dt=1;
time=0:dt:10000;
 
t_count=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------- COORDINATE ENVIRONMENT SIM -------------------%
% Coordinates setup at 1dm distances, and roll at 0.05rads, pitch/yaw
% at 0.1rads
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_coord=[1:dx:x_max-x_min+1];
y_coord=[1:dy:y_max-y_min+1];
zee_coord=[1:dz:z_max-z_min+1];
z_coord(1,1,:)=zee_coord;
roll_coord=[0:droll:2*pi];
pitch_coord=[0:dpitch:2*pi];
yaw_coord=[0:dyaw:2*pi];
% IC's will be a fixed position

% Stores XYZ/RPY of quad-2 wrt time
x_i=zeros(1,length(time));
y_i=zeros(1,length(time));
z_i=zeros(1,length(time));
roll_i=zeros(1,length(time));
pitch_i=zeros(1,length(time));
yaw_i=zeros(1,length(time));


eul = quat2eul([qw_2 qx_2 qy_2 qz_2]); % quat2eul : quaternion to euler angle conversion
roll_i(1,t_count+1) = eul(3); 
pitch_i(1,t_count+1) =  eul(2);
yaw_i(1,t_count+1) = eul(1);


x_i(1,t_count+1)=x_2*10;
y_i(1,t_count+1)=y_2*10;
z_i(1,t_count+1)=z_2*10;

 
%initialize as zero coverage
Q=zeros(length(y_coord),length(x_coord),length(z_coord));
Q_mod=Q;

 
%%%%New on 5/19/16
coverror=zeros(1,length(time));
coverror(1)=1;
ui_save=zeros(1,length(time));
vi_save=zeros(1,length(time));
wi_save=zeros(1,length(time));
ri_save=zeros(1,length(time));
si_save=zeros(1,length(time));
 
 
ui=zeros(1,1);
vi=zeros(1,1);
wi=zeros(1,1);
ri=zeros(1,1);
si=zeros(1,1);
S_i=zeros(length(y_coord),length(x_coord),length(z_coord));



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------------ MAIN COVERAGE LOOP -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%%Main Loop
while K>0
    %Initialize prior belief of next human goal
    for k=1:K
        dist_h_g(k)=1/norm((Human(t,:)-goal(k,:)));
    end
    
    P_g_s=dist_h_g./sum(dist_h_g(:));
    P_g_s=transpose(P_g_s);

    %Compute every possible path through the network and it's path length
    paths = perms(1:K);
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
            Robot_Goals(k,1:length(paths(Sequence,Collab_point+1:K)))=paths(Sequence,Collab_point+1:K);
        end
    else
        Robot_Goals=1;    
    end

    while(1)
        if Capture_Flag==0
            ctrl_msg(1).Type = 2;
            %Cartesian X,Y,Z velocities and Yaw Rate
            ctrl_msg(1).X=0;
            ctrl_msg(1).Y=0;
            ctrl_msg(1).Z=0;
            ctrl_msg(1).Yaw=0;

            %Just an example
            Human(t+1,1)=(x_head-x_orig_ws)*10;
            Human(t+1,2)=(y_head-y_orig_ws)*10;

            %Cycle through each goal
            for k=1:K
                P_a_sg(k,1)=exp(Beta.*(gamma.^(sqrt((Human(t+1,1)-goal(k,1)).^2+(Human(t+1,2)-goal(k,2)).^2))*R-c*((gamma-gamma.^(sqrt((Human(t+1,1)-goal(k,1)).^2+(Human(t+1,2)-goal(k,2)).^2)))/(1-gamma))));
            end
            %Human moves 1 unit towards goal

            %Now Update Distribution
    
            P_g_s = P_a_sg.*P_g_s;
            P_g_s = P_g_s./sum(P_g_s);
         
            
            Robot_Goal_P=zeros(K,1);
            for k=1:K
             %Robot_Goal_P(opt_path(k,K))=Robot_Goal_P(opt_path(k,K))+P_g_s(k);
                Robot_Goal_P(Robot_Goals(k,1))=Robot_Goal_P(Robot_Goals(k,1))+P_g_s(K+1-k);
            end
            Robot_feas_zone=inf*ones(y_max-y_min+1,x_max-x_min+1);
            
            for x=1:x_max-x_min+1
                for y=1:y_max-y_min+1
                    if norm([x-Robot(t,1),y-Robot(t,2)])<=R_max
                        Robot_feas_zone(y,x)=1; 
                    end
                end
            end
          
            Robot_opt_map=zeros(y_max-y_min+1,x_max-x_min+1);
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

            SEND_DEST_X=(x_rob_dest/10);
            SEND_DEST_Y=(y_rob_dest/10);
            Time_save(t+1)=toc;
            z_one(t+1)=z_1;
            
            P_z=arm_thresh-z_one(end);
            D_z=diff(z_one(end-1:end))/diff(Time_save(end-1:end));
            I_z=I_z+(arm_thresh-z_one(end))*diff(Time_save(end-1:end));

            % PID Gains
            %Works good for v_xy and v_z are both 1.0 in quadparameters.yaml
            v_z_prefilt(t)=.1*P_z-.05*D_z+.0001*I_z;
            print=[P_z D_z I_z];
            if t<35
                v_z=v_z_prefilt(end);
            else
                v_z=mean(v_z_prefilt(end-10:end));
            end
           
            v_z_save(t)=v_z;



            heading_vec=[SEND_DEST_X-x_1,SEND_DEST_Y-y_1]./norm([SEND_DEST_X-x_1,SEND_DEST_Y-y_1]);
            v_x=scale*heading_vec(1);
            v_y=scale*heading_vec(2);

            Orient=quat2eul([qw_1 qx_1 qy_1 qz_1]);
            Rot_Mat=[cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
                 cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
                 -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];

            Control_body=Rot_Mat\[v_x;v_y;v_z];
            ctrl_msg(1).X=Control_body(1);
            ctrl_msg(1).Y=Control_body(2);
            ctrl_msg(1).Z=Control_body(3);
            send(ctrl_pub(1),ctrl_msg(1));  


            Robot(t+1,1)=x_1*10;
            Robot(t+1,2)=y_1*10;
            z_one(t+1)=z_1;
            P_save(1:length(P_g_s),t)=P_g_s;




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
                    Capture_Time_Temp=clock;
                    Capture_Time=Capture_Time_Temp(4)*3600+Capture_Time_Temp(5)*60+Capture_Time_Temp(6);
                    Capture_Flag=1;
                    
                end

                clear dist_h_g P_g_s paths opt_dex opt_path P_a_sg possible_collab_lengths possible_collab_temp Robot_Goals
                break
            end

        else
            ctrl_msg(1).Type = 2;
            ctrl_msg(1).X=0;
            ctrl_msg(1).Y=0;
            z_one(t+1)=z_1;
            P_z=arm_thresh-z_one(end);
            D_z=diff(z_one(end-1:end))/diff(Time_save(end-1:end));
            I_z=I_z+(arm_thresh-z_one(end))*diff(Time_save(end-1:end));
            
            % PID Gains
            %WORKS GOOD WHEN max v_xy and v_z are both 0.1 in qua1parameters.yaml
            v_z_prefilt(t)=5*P_z-4*D_z+.005*I_z;
            % PID Gains
            %Works good for v_xy and v_z are both 1.0 in quadparameters.yaml
            v_z_prefilt(t)=.1*P_z-.05*D_z+.0001*I_z;
            print=[P_z D_z I_z];

            if t<35
                v_z=v_z_prefilt(end);
            else
                v_z=mean(v_z_prefilt(end-10:end));
            end
        
            v_z_save(t)=v_z;
            ctrl_msg(1).Z=v_z;     

            send(ctrl_pub(1),ctrl_msg(1));  
            Capture_Time_Temp=clock;
            
            if Capture_Time_Temp(4)*3600+Capture_Time_Temp(5)*60+Capture_Time_Temp(6)>Capture_Time+30
                Capture_Flag=0;
            end
        end


        %%%%%PUT COVERAGE CONTROL CODE HERE%%%%%
        t_count=t_count+1;

        %Coverage Controllers
        x_mat = repmat(x_coord,length(y_coord),1,length(z_coord));
        y_mat = repmat(y_coord',1,length(x_coord),length(z_coord));
        z_mat = repmat(z_coord,length(y_coord),length(x_coord),1);


        %%%%%%%%PLUS DX%%%%%%%%%%%
        x_i(t_count)=x_i(t_count)+dx;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_pdx=Si./max(max(max(Si)));
        x_i(t_count)=x_i(t_count)-dx;
        %%%%%%MINUSDX%%%%%%%
        x_i(t_count)=x_i(t_count)-dx;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_mdx=Si./max(max(max(Si)));
        x_i(t_count)=x_i(t_count)+dx;
        %%%%%%PLUSDY%%%%%%%
        y_i(t_count)=y_i(t_count)+dy;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_pdy=Si./max(max(max(Si)));
        y_i(t_count)=y_i(t_count)-dy;
        %%%%%%MINUSDY%%%%%%%
        y_i(t_count)=y_i(t_count)-dy;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_mdy=Si./max(max(max(Si)));
        y_i(t_count)=y_i(t_count)+dy;
        %%%%%%PLUSDZ%%%%%%%
        z_i(t_count)=z_i(t_count)+dz;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_pdz=Si./max(max(max(Si)));
        z_i(t_count)=z_i(t_count)-dz;
        %%%%%%MINUSDZ%%%%%%%
        z_i(t_count)=z_i(t_count)-dz;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_mdz=Si./max(max(max(Si)));
        z_i(t_count)=z_i(t_count)+dz;
        %%%%%%PLUSDPITCH%%%%%%%
        pitch_i(t_count)=pitch_i(t_count)+dpitch;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_pdpitch=Si./max(max(max(Si)));
        pitch_i(t_count)=pitch_i(t_count)-dpitch;
        %%%%%%MINUSDpitch%%%%%%%
        pitch_i(t_count)=pitch_i(t_count)-dpitch;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_mdpitch=Si./max(max(max(Si)));
        pitch_i(t_count)=pitch_i(t_count)+dpitch;
        %%%%%%PLUSDYaw%%%%%%%
        yaw_i(t_count)=yaw_i(t_count)+dyaw;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_pdyaw=Si./max(max(max(Si)));
        yaw_i(t_count)=yaw_i(t_count)-dyaw;
        %%%%%%MINUSDYaw%%%%%%%
        yaw_i(t_count)=yaw_i(t_count)-dyaw;

        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si_mdyaw=Si./max(max(max(Si)));
        yaw_i(t_count)=yaw_i(t_count)+dyaw;

        %%Nominal
        phi...
            =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
            +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
            (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
            ./sqrt((x_mat-x_i(t_count)).^2....
            +(y_mat-y_i(t_count)).^2....
            +(z_mat-z_i(t_count)).^2)));

        c1=real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
        c2=real(alphaa-phi);
        c3=real(alphaa+phi);
        B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
        Si=1./B;
        Si=Si./max(max(max(Si)));

        Si_dx=(Si_pdx-Si_mdx)./(2*dx);
        Si_dy=(Si_pdy-Si_mdy)./(2*dy);
        Si_dz=(Si_pdz-Si_mdz)./(2*dz);
        Si_dpitch=(Si_pdpitch-Si_mdpitch)./(2*dpitch);
        Si_dyaw=(Si_pdyaw-Si_mdyaw)./(2*dyaw);


        %ai0=trapz(trapz(trapz(6.*max(0,c_star.*C-Q).*...
         %   (Si.^2).*C,1),2),3).*dx.*dy.*dz;
        ai1=trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2.*(...
            Si_dx.*...
            cos(pitch_i(t_count)).*cos(yaw_i(t_count))...
        +Si_dy.*cos(pitch_i(t_count)).*sin(yaw_i(t_count))...
        +Si_dz.*(-sin(pitch_i(t_count))))),1),2),3).*dx.*dy.*dz;
        ai2=trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dx.*(sin(roll_i(t_count)).*sin(pitch_i(t_count)).*cos(yaw_i(t_count))-cos(roll_i(t_count)).*sin(yaw_i(t_count)))+Si_dy.*(sin(roll_i(t_count)).*sin(pitch_i(t_count)).*sin(yaw_i(t_count))+cos(roll_i(t_count)).*cos(yaw_i(t_count)))+Si_dz.*(sin(roll_i(t_count)).*cos(pitch_i(t_count)))),1),2),3).*dx.*dy.*dz;
        ai3=trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dx.*(cos(roll_i(t_count)).*sin(pitch_i(t_count)).*cos(yaw_i(t_count))+sin(roll_i(t_count)).*sin(yaw_i(t_count)))+Si_dy.*(cos(roll_i(t_count)).*sin(pitch_i(t_count)).*sin(yaw_i(t_count))-sin(roll_i(t_count)).*cos(yaw_i(t_count)))+Si_dz.*(cos(roll_i(t_count)).*cos(pitch_i(t_count)))),1),2),3).*dx.*dy.*dz;
        ai4=trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dyaw.*(sin(roll_i(t_count)).*sec(pitch_i(t_count)))+Si_dpitch.*cos(roll_i(t_count))),1),2),3).*dx.*dy.*dz;
        ai5=trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dyaw.*(cos(roll_i(t_count)).*sec(pitch_i(t_count)))+Si_dpitch.*(-sin(roll_i(t_count)))),1),2),3).*dx.*dy.*dz;
        
        % if any of the above is NaN, replace with 0
        ai1(isnan(ai1))=0;
        ai2(isnan(ai2))=0;
        ai3(isnan(ai3))=0;
        ai4(isnan(ai4))=0;
        ai5(isnan(ai5))=0;

        %Propagate Agent State
        eul=quat2eul([qw_2 qx_2 qy_2 qz_2]);
        roll_i(1,t_count+1) = eul(3); 
        pitch_i(1,t_count+1) =  eul(2);
        yaw_i(1,t_count+1) =eul(1);


        x_i(1,t_count+1)=x_2*10;
        y_i(1,t_count+1)=y_2*10;
        z_i(1,t_count+1)=z_2*10;

        if x_i(t_count+1)>max(x_coord)
            x_i(t_count+1)=max(x_coord);
        end
        if x_i(t_count+1)<min(x_coord)
            x_i(t_count+1)=min(x_coord);
        end
        
        if y_i(t_count+1)>max(y_coord)
            y_i(t_count+1)=max(y_coord);
        end
        if y_i(t_count+1)<min(y_coord)
            y_i(t_count+1)=min(y_coord);
        end
        
        if z_i(t_count+1)>max(z_coord)
            z_i(t_count+1)=max(z_coord);
        end
        if z_i(t_count+1)<min(z_coord)
            z_i(t_count+1)=min(z_coord);
        end  

        ui(t_count)=K_u*ai1;
        vi(t_count)=K_v*ai2;
        wi(t_count)=K_w*ai3;
        qi(t_count)=0;
        ri(t_count)=K_r*ai4;
        si(t_count)=K_s*ai5;


        Rot_Mat=[cos(pitch_i(t_count))*cos(yaw_i(t_count)),(sin(roll_i(t_count))*sin(pitch_i(t_count))*cos(yaw_i(t_count))-cos(roll_i(t_count))*sin(yaw_i(t_count))),(cos(roll_i(t_count))*sin(pitch_i(t_count))*cos(yaw_i(t_count))+sin(roll_i(t_count))*sin(yaw_i(t_count))); ...
                 cos(pitch_i(t_count))*sin(yaw_i(t_count)),(sin(roll_i(t_count))*sin(pitch_i(t_count))*sin(yaw_i(t_count))+cos(roll_i(t_count))*cos(yaw_i(t_count))),(cos(roll_i(t_count))*sin(pitch_i(t_count))*sin(yaw_i(t_count))-sin(roll_i(t_count))*cos(yaw_i(t_count))); ...
                 -sin(pitch_i(t_count)),(sin(roll_i(t_count))*cos(pitch_i(t_count))),(cos(roll_i(t_count))*cos(pitch_i(t_count)))];

        bound_vx=[0;0;0];
        bound_vy=[0;0;0];
        bound_vz=[0;0;0];
        bound_vc=[0;0;0];
        bound_flag=0;
    
        if x_i(t_count+1)==max(x_coord)
            bound_vx=Rot_Mat\[-U_max;0;0];
            bound_flag=1;
        end
        if x_i(t_count+1)==min(x_coord)
            bound_vx=Rot_Mat\[U_max;0;0];
            bound_flag=1;
        end
        
        if y_i(t_count+1)==max(y_coord)
            bound_vy=Rot_Mat\[0;-U_max;0]; 
            bound_flag=1;
        end
        if y_i(t_count+1)==min(y_coord)
            bound_vy=Rot_Mat\[0;U_max;0]; 
            bound_flag=1;
        end
        
        if z_i(t_count+1)==max(z_coord)
            bound_vz=Rot_Mat\[0;0;-U_max]; 
            bound_flag=1;
        end
        if z_i(t_count+1)==min(z_coord)
            bound_vz=Rot_Mat\[0;0;U_max]; 
            bound_flag=1;
        end
       if bound_flag==1
           bound_vc=bound_vx+bound_vy+bound_vz;
           ui(t_count)=bound_vc(1);
           vi(t_count)=bound_vc(2);
           wi(t_count)=bound_vc(3);
       end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------------------------- GESTURE CONTROL --------------------------%
        % Follow the arm to a position it points at if right raised & left not
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        fprintf("Right Hand: %f, Left Hand: %f\n",z_right, z_left);
        
        if z_right>arm_thresh && z_left<arm_thresh

            dist_me=norm([x_head - (x_orig_ws + x_2) ,y_head- (y_orig_ws + y_2)]);
            fprintf("head: (%f, %f, %f), hbirddg: (%f, %f, %f), dist:%f \n", x_head, y_head, z_head, x_2, y_2, z_2, dist_me);
            
            if z_2<arm_thresh
               v_z = scale;
            else
               v_z = -scale;
            end

            if dist_me<1
                heading_vec=-[x_head - (x_orig_ws + x_2) ,y_head- (y_orig_ws + y_2)]./dist_me;
                v_x=scale*heading_vec(1);
                v_y=scale*heading_vec(2);
            end

            if dist_me>1
                heading_vec=[x_head - (x_orig_ws + x_2) ,y_head- (y_orig_ws + y_2)]./dist_me;
                v_x=scale*heading_vec(1);
                v_y=scale*heading_vec(2);
            end


            Orient=quat2eul([qw_2 qx_2 qy_2 qz_2]);
            Rot_Mat=[cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
                     cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
                     -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];

            Control_body=Rot_Mat\[v_x;v_y;v_z];
            ui(t_count)=Control_body(1);
            vi(t_count)=Control_body(2);
            wi(t_count)=Control_body(3);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------------------------- GESTURE CONTROL --------------------------%
        % Come close to the operator and hover if left arm raised and right not
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if z_left>arm_thresh
            pos_rel_arm=[x_2-x_right-x_orig_ws; y_2-y_right-y_orig_ws; z_2-z_right-z_orig_ws];
            Orient_right=quat2eul([qw_right qx_right qy_right qz_right]);
            Rot_Mat_right=[cos(Orient_right(2))*cos(Orient_right(1)),(sin(Orient_right(3))*sin(Orient_right(2))*cos(Orient_right(1))-cos(Orient_right(3))*sin(Orient_right(1))),(cos(Orient_right(3))*sin(Orient_right(2))*cos(Orient_right(1))+sin(Orient_right(3))*sin(Orient_right(1))); ...
             cos(Orient_right(2))*sin(Orient_right(1)),(sin(Orient_right(3))*sin(Orient_right(2))*sin(Orient_right(1))+cos(Orient_right(3))*cos(Orient_right(1))),(cos(Orient_right(3))*sin(Orient_right(2))*sin(Orient_right(1))-sin(Orient_right(3))*cos(Orient_right(1))); ...
             -sin(Orient_right(2)),(sin(Orient_right(3))*cos(Orient_right(2))),(cos(Orient_right(3))*cos(Orient_right(2)))];
            pos_rel_arm=Rot_Mat_right\pos_rel_arm; %Now in the arm frame
            v_x=scale;
            v_y=-scale*pos_rel_arm(2);
            v_z=-scale*pos_rel_arm(3);
            Back_to_room=Rot_Mat_right*[v_x;v_y;v_z];

            Orient=quat2eul([qw_2 qx_2 qy_2 qz_2]);
            Rot_Mat=[cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
             cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
             -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];
            Control_body=Rot_Mat\Back_to_room;
            ui(t_count)=Control_body(1);
            vi(t_count)=Control_body(2);
            wi(t_count)=Control_body(3);     
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------------- Generate control message for quad 2 ------------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        ctrl_msg(2).Type = 2;
        ctrl_msg(2).X = ui(t_count);
        ctrl_msg(2).Y = vi(t_count);
        ctrl_msg(2).Z = wi(t_count);
        ctrl_msg(2).Yaw = si(t_count);
        send(ctrl_pub(2),ctrl_msg(2));  
        disp("publishing here for quad 2");

        %Propogate Coverage Level
        if t_count==1
            dttt=0.1;
        else
            dttt=toc;
        end

        Si(isnan(Si))=0;
        Q=Q+Si*dttt;
        tic
        Q_mod=Q;

        curr_x_Indx_Num=int64(((1/dx)*(Robot(t,1)+dx)));
        curr_y_Indx_Num=int64(((1/dy)*(Robot(t,2)+dy)));
        curr_z_Indx_Num=int64(((1/dz)*(z_1*10+dz)));
        x_left_1=max(1,curr_x_Indx_Num-1*(Ri+2*r_i)/dx);
        x_right_1=min(length(x_coord),curr_x_Indx_Num+1*(Ri+2*r_i)/dx);
        y_left_1=max(1,curr_y_Indx_Num-1*(Ri+2*r_i)/dy);
        y_right_1=min(length(y_coord),curr_y_Indx_Num+1*(Ri+2*r_i)/dy);
        z_left_1=max(1,curr_z_Indx_Num-1*(Ri+2*r_i)/dz);
        z_right_1=min(length(z_coord),curr_z_Indx_Num+1*(Ri+2*r_i)/dz);

        Q_mod(y_left_1:y_right_1,x_left_1:x_right_1,z_left_1:z_right_1)=c_star;
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Make Scatter Plot of the Simulation
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if t_count==1
            coverror_init=trapz(trapz(trapz(max(0,c_star).^3)))*dx*dy*dz;
            [xx,yy,zz]=meshgrid(x_min:3:x_max,y_min:3:y_max,z_min:3:z_max);
            temp=max(0,c_star-Q(1:3:y_max-y_min+1,1:3:x_max-x_min+1,1:3:z_max-z_min+1));
            tempp=temp(:);
            h=scatter3(xx(:),yy(:),zz(:),1,tempp,'linewidth',10);
            h.CDataSource='tempp';
            caxis([0 c_star-1])
            view([80,15])
            xlim([x_min, x_max])
            ylim([y_min, y_max])
            zlim([z_min, z_max])
            xlabel('x')
            ylabel('y')
            zlabel('z')
            alpha(0.1);
        end
        
        temp=max(0,c_star-Q(1:3:y_max-y_min+1,1:3:x_max-x_min+1,1:3:z_max-z_min+1));  
        tempp=temp(:);
        refreshdata(h)
        coverror(t_count)=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
        coverror(t_count)=coverror(t_count)/coverror_init;
            
        %%%%%END COVERAGE CONTROL CODE HERE%%%%%
        Human(t+1,1) = (x_head-x_orig_ws)*10;
        Human(t+1,2) = (y_head-y_orig_ws)*10;
        Robot(t+1,1) = x_1*10;
        Robot(t+1,2) = y_1*10;
        z_one(t+1) = z_1;
        t=t+1;
        drawnow
    end

    Final_Time=Capture_List(end,end);
    %End of outer loop
end

clear global qx qy qz qw x y z;
rosshutdown;
