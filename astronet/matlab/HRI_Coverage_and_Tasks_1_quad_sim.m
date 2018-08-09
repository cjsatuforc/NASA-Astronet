rosshutdown, clc;
close all
tic

% Global variables
% Quaternion
global qx_1 qx_head qx_left qx_right;
global qy_1 qy_head qy_left qy_right;
global qz_1 qz_head qz_left qz_right;
global qw_1 qw_head qw_left qw_right;

% Position
global x_1 x_head x_left x_right;
global y_1 y_head y_left y_right;
global z_1 z_head z_left z_right;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------- ROS parameters - Astronet -----------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setenv('ROS_MASTER_URI', 'http://localhost:11311');
rosinit('NodeName', '/matlab');

quad_list = {'/hbirdbState' '/vicon/Oculus/Oculus' '/vicon/Left_Arm/Left_Arm' '/vicon/Right_Arm/Right_Arm'};
viconCallback_list = {@viconCallback_hbirdb @viconCallback_Oculus @viconCallback_Left_Arm @viconCallback_Right_Arm};

quad_num = 1;
tic

% subscribe to gazebo states of the models of the quad
vicon_sub(1) = rossubscriber(quad_list{1},'geometry_msgs/Transform',viconCallback_list{1});

% subscribe to the head, left_arm, right_arm positions published by Vicon
vicon_sub(2) = rossubscriber(quad_list{2},'geometry_msgs/TransformStamped',viconCallback_list{2});
vicon_sub(3) = rossubscriber(quad_list{3},'geometry_msgs/TransformStamped',viconCallback_list{3});
vicon_sub(4) = rossubscriber(quad_list{4},'geometry_msgs/TransformStamped',viconCallback_list{4});

% Define loop rate
rate = rosrate(60);

% publish control messages to the quad
ctrl_pub(1) = rospublisher(sprintf('/cov_ctrl_1'),'asctec_hl_comm/mav_ctrl');
ctrl_msg(1) = rosmessage(ctrl_pub(1));

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
%-------------------- SIM ENVIRONMENT PARAMETERS ---------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_min = -8; x_max = 50;
y_min = -4; y_max = 6;
z_min = 43; z_max = 50;

% Set origin for the simulator
% to be subtracted from simulator data
x_orig = (x_min)/10;
y_orig = (y_min)/10;
z_orig = (z_min)/10;

% Set origin for the actual physical workspace
% to be subtracted from vicon data
x_orig_ws = 0;
y_orig_ws = 0;
z_orig_ws = 1.5;    % set the origin to the height of the operators shoulder

% thresholds
arm_thresh = 1.3;   % arm height threshold
dist_thresh = 0.05;  % distance threshold from wall to avoid clash
v_thresh = 1e-5;    % threshold velocity below which bias velocity kicks in
v_bias = 5e-3;      % bias velocity applied

% defining z-velocity(was set to 0.2 everywhere previously) and commanded linear/angular velocity scale
vz_scale = 0.2;
vel_scale = 1;
ang_scale = 1;

% meshgrid dimensions for scatter plot
dxx = 3; dyy = 1; dzz = 1;

fprintf("Gazebo Origin: (%f, %f, %f)\nWorkspace Origin: (%f, %f, %f)", x_orig, y_orig, z_orig, x_orig_ws, y_orig_ws, z_orig_ws);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------------- SIM ENVIRONMENT DIMS ------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[x_mesh,y_mesh] = meshgrid(1:x_max-x_min+1,1:y_max-y_min+1);
t=1;

%Human Starting Location
Human(t,1)=(x_head-x_orig_ws)*10;
Human(t,2)=(y_head-y_orig_ws)*10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------------- NORMALIZE POSITIONS -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Robot Starting Location
Robot(t,1) = (x_1 - x_orig)*10;
Robot(t,2) = (y_1 - y_orig)*10;
z_one(1) = (z_1 - z_orig)*10;

Capture_Flag = 0;
Time_save(1) = 0;

D_z = 0; I_z = 0; P_z = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------  SETUP COVERAGE PARAMETERS -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set up Coverage Parameters
c_star = 1;

% simulation gains
K_u = .05; K_v = .05; K_w = .05; K_r = 3e-3; K_s = 3e-3; K_vel = 1;

U_max = .2; r_sat = 0.02; s_sat = 0.02;
r_i = 3;
r_part = 2;
%Size of Sensor Footprint
Ri = 5;
alphaa = (1/2)*90*pi/180;
update_count = 0;
orient_tol = .01;
Ep_star = .05;


%set up coordinates
%NOminal are .5 and .1....Works well!
dx = 1; dy = 1; dz = 1; droll = .05; dpitch = dx/Ri; dyaw = dx/Ri;
%set(gcf,'visible','off')
dt = 1;
time = 0:dt:10000;

t_count = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------- COORDINATE ENVIRONMENT SIM -------------------%
% Coordinates setup at 1dm distances, and roll at 0.05rads, pitch/yaw
% at 0.1rads
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_coord = [1:dx:x_max-x_min+1];
y_coord = [1:dy:y_max-y_min+1];
zee_coord = [1:dz:z_max-z_min+1];
z_coord(1,1,:) = zee_coord;
roll_coord = [0:droll:2*pi];
pitch_coord = [0:dpitch:2*pi];
yaw_coord = [0:dyaw:2*pi];
% IC's will be a fixed position

% Stores XYZ/RPY of quad-2 wrt time
x_i = zeros(1,length(time));
y_i = zeros(1,length(time));
z_i = zeros(1,length(time));
roll_i = zeros(1,length(time));
pitch_i = zeros(1,length(time));
yaw_i = zeros(1,length(time));


eul = quat2eul([qw_1 qx_1 qy_1 qz_1]); % quat2eul : quaternion to euler angle conversion
roll_i(1,t_count+1) = eul(3);
pitch_i(1,t_count+1) =  eul(2);
yaw_i(1,t_count+1) = eul(1);


x_i(1,t_count+1) = (x_1 - x_orig)*10;
y_i(1,t_count+1) = (y_1 - y_orig)*10;
z_i(1,t_count+1) = (z_1 - z_orig)*10;


%initialize as zero coverage
Q = zeros(length(y_coord),length(x_coord),length(z_coord));
Q_mod = Q;


%%%%New on 5/19/16
coverror = zeros(1,length(time));
coverror(1) = 1;
ui_save = zeros(1,length(time));
vi_save = zeros(1,length(time));
wi_save = zeros(1,length(time));
ri_save = zeros(1,length(time));
si_save = zeros(1,length(time));


ui = zeros(1,1);
vi = zeros(1,1);
wi = zeros(1,1);
ri = zeros(1,1);
si = zeros(1,1);
S_i = zeros(length(y_coord),length(x_coord),length(z_coord));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------------ MAIN COVERAGE LOOP -------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%Main Loop
while(1)
     %%%%%PUT COVERAGE CONTROL CODE HERE%%%%%
    t_count = t_count+1;
    x_i(t_count) = (x_1 - x_orig)*10;
    y_i(t_count) = (y_1 - y_orig)*10;
    z_i(t_count) = (z_1 - z_orig)*10;

    %Coverage Controllers
    x_mat = repmat(x_coord,length(y_coord),1,length(z_coord));
    y_mat = repmat(y_coord',1,length(x_coord),length(z_coord));
    z_mat = repmat(z_coord,length(y_coord),length(x_coord),1);


    %%%%%%%%PLUS DX%%%%%%%%%%%
    x_i(t_count) = x_i(t_count)+dx;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_pdx = Si./max(max(max(Si)));
    x_i(t_count) = x_i(t_count)-dx;
    %%%%%%MINUSDX%%%%%%%
    x_i(t_count) = x_i(t_count)-dx;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_mdx = Si./max(max(max(Si)));
    x_i(t_count) = x_i(t_count)+dx;
    %%%%%%PLUSDY%%%%%%%
    y_i(t_count) = y_i(t_count)+dy;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_pdy = Si./max(max(max(Si)));
    y_i(t_count) = y_i(t_count)-dy;
    %%%%%%MINUSDY%%%%%%%
    y_i(t_count) = y_i(t_count)-dy;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_mdy = Si./max(max(max(Si)));
    y_i(t_count) = y_i(t_count)+dy;
    %%%%%%PLUSDZ%%%%%%%
    z_i(t_count) = z_i(t_count)+dz;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_pdz = Si./max(max(max(Si)));
    z_i(t_count) = z_i(t_count)-dz;
    %%%%%%MINUSDZ%%%%%%%
    z_i(t_count) = z_i(t_count)-dz;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_mdz = Si./max(max(max(Si)));
    z_i(t_count) = z_i(t_count)+dz;
    %%%%%%PLUSDPITCH%%%%%%%
    pitch_i(t_count) = pitch_i(t_count)+dpitch;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_pdpitch = Si./max(max(max(Si)));
    pitch_i(t_count) = pitch_i(t_count)-dpitch;
    %%%%%%MINUSDpitch%%%%%%%
    pitch_i(t_count) = pitch_i(t_count)-dpitch;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_mdpitch = Si./max(max(max(Si)));
    pitch_i(t_count) = pitch_i(t_count)+dpitch;
    %%%%%%PLUSDYaw%%%%%%%
    yaw_i(t_count) = yaw_i(t_count)+dyaw;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_pdyaw = Si./max(max(max(Si)));
    yaw_i(t_count) = yaw_i(t_count)-dyaw;
    %%%%%%MINUSDYaw%%%%%%%
    yaw_i(t_count) = yaw_i(t_count)-dyaw;

    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si_mdyaw = Si./max(max(max(Si)));
    yaw_i(t_count) = yaw_i(t_count)+dyaw;

    %%Nominal
    phi...
        =acos((((x_mat-x_i(t_count)).*cos(yaw_i(t_count)).*cos(pitch_i(t_count))...
        +(y_mat-y_i(t_count)).*sin(yaw_i(t_count)).*cos(pitch_i(t_count))-...
        (z_mat-z_i(t_count)).*sin(pitch_i(t_count)))...
        ./sqrt((x_mat-x_i(t_count)).^2....
        +(y_mat-y_i(t_count)).^2....
        +(z_mat-z_i(t_count)).^2)));

    c1 = real(Ri^2-(x_mat-x_i(t_count)).^2-(y_mat-y_i(t_count)).^2-(z_mat-z_i(t_count)).^2);
    c2 = real(alphaa-phi);
    c3 = real(alphaa+phi);
    B = (1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
    Si = 1./B;
    Si = Si./max(max(max(Si)));

    Si_dx = (Si_pdx-Si_mdx)./(2*dx);
    Si_dy = (Si_pdy-Si_mdy)./(2*dy);
    Si_dz = (Si_pdz-Si_mdz)./(2*dz);
    Si_dpitch = (Si_pdpitch-Si_mdpitch)./(2*dpitch);
    Si_dyaw = (Si_pdyaw-Si_mdyaw)./(2*dyaw);


    %ai0=trapz(trapz(trapz(6.*max(0,c_star.*C-Q).*...
     %   (Si.^2).*C,1),2),3).*dx.*dy.*dz;
    ai1 = trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2.*(...
        Si_dx.*...
        cos(pitch_i(t_count)).*cos(yaw_i(t_count))...
    +Si_dy.*cos(pitch_i(t_count)).*sin(yaw_i(t_count))...
    +Si_dz.*(-sin(pitch_i(t_count))))),1),2),3).*dx.*dy.*dz;
    ai2 = trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dx.*(sin(roll_i(t_count)).*sin(pitch_i(t_count)).*cos(yaw_i(t_count))-cos(roll_i(t_count)).*sin(yaw_i(t_count)))+Si_dy.*(sin(roll_i(t_count)).*sin(pitch_i(t_count)).*sin(yaw_i(t_count))+cos(roll_i(t_count)).*cos(yaw_i(t_count)))+Si_dz.*(sin(roll_i(t_count)).*cos(pitch_i(t_count)))),1),2),3).*dx.*dy.*dz;
    ai3 = trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dx.*(cos(roll_i(t_count)).*sin(pitch_i(t_count)).*cos(yaw_i(t_count))+sin(roll_i(t_count)).*sin(yaw_i(t_count)))+Si_dy.*(cos(roll_i(t_count)).*sin(pitch_i(t_count)).*sin(yaw_i(t_count))-sin(roll_i(t_count)).*cos(yaw_i(t_count)))+Si_dz.*(cos(roll_i(t_count)).*cos(pitch_i(t_count)))),1),2),3).*dx.*dy.*dz;
    ai4 = trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dyaw.*(sin(roll_i(t_count)).*sec(pitch_i(t_count)))+Si_dpitch.*cos(roll_i(t_count))),1),2),3).*dx.*dy.*dz;
    ai5 = trapz(trapz(trapz((3.*max(0,c_star-Q_mod).^2).*(Si_dyaw.*(cos(roll_i(t_count)).*sec(pitch_i(t_count)))+Si_dpitch.*(-sin(roll_i(t_count)))),1),2),3).*dx.*dy.*dz;

    % if any of the above is NaN, replace with 0
    ai1(isnan(ai1)) = 0;
    ai2(isnan(ai2)) = 0;
    ai3(isnan(ai3)) = 0;
    ai4(isnan(ai4)) = 0;
    ai5(isnan(ai5)) = 0;

    %Propagate Agent State
    eul = quat2eul([qw_1 qx_1 qy_1 qz_1]);
    roll_i(1,t_count+1) = eul(3);
    pitch_i(1,t_count+1) =  eul(2);
    yaw_i(1,t_count+1) = eul(1);


    x_i(1,t_count+1) = (x_1 - x_orig)*10;
    y_i(1,t_count+1) = (y_1 - y_orig)*10;
    z_i(1,t_count+1) = (z_1 - z_orig)*10;

    if x_i(t_count+1)>max(x_coord)
        x_i(t_count+1) = max(x_coord);
    end
    if x_i(t_count+1)<min(x_coord)
        x_i(t_count+1) = min(x_coord);
    end

    if y_i(t_count+1)>max(y_coord)
        y_i(t_count+1) = max(y_coord);
    end
    if y_i(t_count+1)<min(y_coord)
        y_i(t_count+1) = min(y_coord);
    end

    if z_i(t_count+1)>max(z_coord)
        z_i(t_count+1) = max(z_coord);
    end
    if z_i(t_count+1)<min(z_coord)
        z_i(t_count+1) = min(z_coord);
    end

    ui(t_count) = K_u*ai1;
    vi(t_count) = K_v*ai2;
    wi(t_count) = K_w*ai3;
    qi(t_count) = 0;
    ri(t_count) = K_r*ai4; % pitch
    si(t_count) = K_s*ai5; % yaw


    Rot_Mat = [cos(pitch_i(t_count))*cos(yaw_i(t_count)),(sin(roll_i(t_count))*sin(pitch_i(t_count))*cos(yaw_i(t_count))-cos(roll_i(t_count))*sin(yaw_i(t_count))),(cos(roll_i(t_count))*sin(pitch_i(t_count))*cos(yaw_i(t_count))+sin(roll_i(t_count))*sin(yaw_i(t_count))); ...
             cos(pitch_i(t_count))*sin(yaw_i(t_count)),(sin(roll_i(t_count))*sin(pitch_i(t_count))*sin(yaw_i(t_count))+cos(roll_i(t_count))*cos(yaw_i(t_count))),(cos(roll_i(t_count))*sin(pitch_i(t_count))*sin(yaw_i(t_count))-sin(roll_i(t_count))*cos(yaw_i(t_count))); ...
             -sin(pitch_i(t_count)),(sin(roll_i(t_count))*cos(pitch_i(t_count))),(cos(roll_i(t_count))*cos(pitch_i(t_count)))];

    bound_vx = [0;0;0];
    bound_vy = [0;0;0];
    bound_vz = [0;0;0];
    bound_vc = [0;0;0];
    bound_flag = 0;

    if x_i(t_count+1)==max(x_coord)
        bound_vx = Rot_Mat\[-U_max;0;0];
        bound_flag = 1;
    end
    if x_i(t_count+1)==min(x_coord)
        bound_vx = Rot_Mat\[U_max;0;0];
        bound_flag = 1;
    end

    if y_i(t_count+1)==max(y_coord)
        bound_vy = Rot_Mat\[0;-U_max;0];
        bound_flag = 1;
    end
    if y_i(t_count+1)==min(y_coord)
        bound_vy = Rot_Mat\[0;U_max;0];
        bound_flag = 1;
    end

    if z_i(t_count+1)==max(z_coord)
        bound_vz = Rot_Mat\[0;0;-U_max];
        bound_flag = 1;
    end
    if z_i(t_count+1)==min(z_coord)
        bound_vz = Rot_Mat\[0;0;U_max];
        bound_flag = 1;
    end
   % if near boundary, send back
   if bound_flag==1
       bound_vc = bound_vx+bound_vy+bound_vz;
       ui(t_count) = bound_vc(1);
       vi(t_count) = bound_vc(2);
       wi(t_count) = bound_vc(3);
   end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %-------------------------- GESTURE CONTROL --------------------------%
    %---------------- Come to daddy and hover near my head ---------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Calculate distances to all the walls
    dist_x_min = abs(x_1 - x_min/10); dist_x_max = abs(x_1 - x_max/10);
    dist_y_min = abs(y_1 - y_min/10); dist_y_max = abs(y_1 - y_max/10);
    dist_z_min = abs(z_1 - z_min/10); dist_z_max = abs(z_1 - z_max/10);

    if z_right>arm_thresh && z_left<arm_thresh
        disp("come here");
        % fprintf("Wall Distances: min - (%f,%f,%f) | max - (%f,%f,%f)\n", ...
        % dist_x_min, dist_y_min, dist_z_min, dist_x_max, dist_y_max, dist_z_max);

        dist_me = norm([(x_head-x_orig_ws) - (x_1 - x_orig) ,(y_head-y_orig_ws) - (y_1 - y_orig_ws)]);

        if (z_1 - z_orig)< (z_head - z_orig_ws)*0.6
           v_z = 0.1;
        else
           v_z = -0.1;
        end

        if dist_me<1
            heading_vec = -[(x_head-x_orig_ws) - (x_1 - x_orig) ,(y_head-y_orig_ws) - (y_1 - y_orig_ws)]./dist_me;
            v_x = vz_scale*heading_vec(1);
            v_y = vz_scale*heading_vec(2);
        end

        if dist_me>1
            heading_vec = [(x_head-x_orig_ws) - (x_1 - x_orig) ,(y_head-y_orig_ws) - (y_1 - y_orig_ws)]./dist_me;
            v_x = vz_scale*heading_vec(1);
            v_y = vz_scale*heading_vec(2);
        end

        Orient = quat2eul([qw_1 qx_1 qy_1 qz_1]);
        Rot_Mat = [cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
                 cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
                 -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];

        Control_body = Rot_Mat\[v_x;v_y;v_z];

        % add bias velocity if astrobee near the walls
        if dist_x_min<dist_thresh
            Control_body(1) = (exp(abs(dist_thresh-dist_x_min))-1);
            fprintf("min_x: %f, v: %f\n",dist_x_min, Control_body(1));
        end
        if dist_x_max<dist_thresh
            Control_body(1) = -(exp(abs(dist_thresh-dist_x_max))-1);
            fprintf("max_x: %f, v: %f\n",dist_x_max, Control_body(1));
        end

        if dist_y_min<dist_thresh
            Control_body(2) = (exp(abs(dist_thresh-dist_y_min))-1);
            fprintf("min_y: %f, v: %f\n",dist_y_min, Control_body(2));
        end
        if dist_y_max<dist_thresh
            Control_body(2) = -(exp(abs(dist_thresh-dist_y_max))-1);
            fprintf("max_y: %f, v: %f\n",dist_y_max, Control_body(2));
        end

        if dist_z_min<dist_thresh
            Control_body(3) = (exp(abs(dist_thresh-dist_z_min))-1);
            fprintf("min_z: %f, v: %f\n",dist_z_min, Control_body(3));
        end
        if dist_z_max<dist_thresh
            Control_body(3) = -(exp(abs(dist_thresh-dist_z_max))-1);
            fprintf("max_z: %f, v: %f\n",dist_z_max, Control_body(3));
        end

        ui(t_count) = Control_body(1);
        vi(t_count) = Control_body(2);
        wi(t_count) = Control_body(3);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %-------------------------- GESTURE CONTROL --------------------------%
    %---------------- Follow the hand vector till position ---------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if z_left>arm_thresh
        disp("follow vector");
        % fprintf("Wall Distances: min - (%f,%f,%f) | max - (%f,%f,%f)\n", ...
        % dist_x_min, dist_y_min, dist_z_min, dist_x_max, dist_y_max, dist_z_max);

        pos_rel_arm = [(x_1 - x_orig) - (x_right - x_orig_ws);...
                       (y_1 - y_orig) - (y_right - y_orig_ws);...
                       (z_1 - z_orig) - (z_right - z_orig_ws)];

        Orient_right = quat2eul([qw_right qx_right qy_right qz_right]);
        Rot_Mat_right = [cos(Orient_right(2))*cos(Orient_right(1)),(sin(Orient_right(3))*sin(Orient_right(2))*cos(Orient_right(1))-cos(Orient_right(3))*sin(Orient_right(1))),(cos(Orient_right(3))*sin(Orient_right(2))*cos(Orient_right(1))+sin(Orient_right(3))*sin(Orient_right(1))); ...
         cos(Orient_right(2))*sin(Orient_right(1)),(sin(Orient_right(3))*sin(Orient_right(2))*sin(Orient_right(1))+cos(Orient_right(3))*cos(Orient_right(1))),(cos(Orient_right(3))*sin(Orient_right(2))*sin(Orient_right(1))-sin(Orient_right(3))*cos(Orient_right(1))); ...
         -sin(Orient_right(2)),(sin(Orient_right(3))*cos(Orient_right(2))),(cos(Orient_right(3))*cos(Orient_right(2)))];
        pos_rel_arm = Rot_Mat_right\pos_rel_arm; %Now in the arm frame

        v_x = vz_scale;
        v_y = -vz_scale*pos_rel_arm(2);
        v_z = -vz_scale*pos_rel_arm(3);
        Back_to_room = Rot_Mat_right*[v_x;v_y;v_z];

        Orient = quat2eul([qw_1 qx_1 qy_1 qz_1]);
        Rot_Mat = [cos(Orient(2))*cos(Orient(1)),(sin(Orient(3))*sin(Orient(2))*cos(Orient(1))-cos(Orient(3))*sin(Orient(1))),(cos(Orient(3))*sin(Orient(2))*cos(Orient(1))+sin(Orient(3))*sin(Orient(1))); ...
         cos(Orient(2))*sin(Orient(1)),(sin(Orient(3))*sin(Orient(2))*sin(Orient(1))+cos(Orient(3))*cos(Orient(1))),(cos(Orient(3))*sin(Orient(2))*sin(Orient(1))-sin(Orient(3))*cos(Orient(1))); ...
         -sin(Orient(2)),(sin(Orient(3))*cos(Orient(2))),(cos(Orient(3))*cos(Orient(2)))];

        Control_body = Rot_Mat\Back_to_room;

        % add bias velocity if astrobee near the walls
        if dist_x_min<dist_thresh
            Control_body(1) = (exp(abs(dist_thresh-dist_x_min))-1);
            fprintf("min_x: %f, v: %f\n",dist_x_min, Control_body(1));
        end
        if dist_x_max<dist_thresh
            Control_body(1) = -(exp(abs(dist_thresh-dist_x_max))-1);
            fprintf("max_x: %f, v: %f\n",dist_x_max, Control_body(1));
        end

        if dist_y_min<dist_thresh
            Control_body(2) = (exp(abs(dist_thresh-dist_y_min))-1);
            fprintf("min_y: %f, v: %f\n",dist_y_min, Control_body(2));
        end
        if dist_y_max<dist_thresh
            Control_body(2) = -(exp(abs(dist_thresh-dist_y_max))-1);
            fprintf("max_y: %f, v: %f\n",dist_y_max, Control_body(2));
        end

        if dist_z_min<dist_thresh
            Control_body(3) = (exp(abs(dist_thresh-dist_z_min))-1);
            fprintf("min_z: %f, v: %f\n",dist_z_min, Control_body(3));
        end
        if dist_z_max<dist_thresh
            Control_body(3) = -(exp(abs(dist_thresh-dist_z_max))-1);
            fprintf("max_z: %f, v: %f\n",dist_z_max, Control_body(3));
        end

        ui(t_count) = Control_body(1);
        vi(t_count) = Control_body(2);
        wi(t_count) = Control_body(3);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %-------------- Generate control message for quad 2 ------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    ctrl_msg(1).Type = 2;

    % publishing velocities
    ctrl_msg(1).X = vel_scale * ui(t_count);
    ctrl_msg(1).Y = vel_scale * vi(t_count);
    ctrl_msg(1).Z = vel_scale * wi(t_count);

    % publishing pitch and yaw
    ctrl_msg(1).Yaw = ang_scale * si(t_count);
    ctrl_msg(1).VMaxZ = ang_scale * ri(t_count);

    % add momentum if stuck in local minima
    if(abs(ctrl_msg(1).X)<v_thresh)
        ctrl_msg(1).X = v_bias;
    end
    if(abs(ctrl_msg(1).Y)<v_thresh)
        ctrl_msg(1).Y = v_bias;
    end
    if(abs(ctrl_msg(1).Z)<v_thresh)
        ctrl_msg(1).Z = v_bias;
    end


    send(ctrl_pub(1),ctrl_msg(1));
    fprintf("Vel:(%f, %f, %f), Ang:(0, %f, %f)\n",ctrl_msg(1).X, ctrl_msg(1).Y, ctrl_msg(1).Z, ctrl_msg(1).VMaxZ, ctrl_msg(1).Yaw);

    %Propogate Coverage Level
    if t_count==1
        dttt = 0.1;
    else
        dttt = toc;
    end

    Si(isnan(Si)) = 0;
    Q = Q+Si*dttt;
    tic
    Q_mod = Q;

    curr_x_Indx_Num = int64(((1/dx)*(Robot(t,1)+dx)));
    curr_y_Indx_Num = int64(((1/dy)*(Robot(t,2)+dy)));
    curr_z_Indx_Num = int64(((1/dz)*(z_1*10+dz)));
    x_left_1 = max(1,curr_x_Indx_Num-1*(Ri+2*r_i)/dx);
    x_right_1 = min(length(x_coord),curr_x_Indx_Num+1*(Ri+2*r_i)/dx);
    y_left_1 = max(1,curr_y_Indx_Num-1*(Ri+2*r_i)/dy);
    y_right_1 = min(length(y_coord),curr_y_Indx_Num+1*(Ri+2*r_i)/dy);
    z_left_1 = max(1,curr_z_Indx_Num-1*(Ri+2*r_i)/dz);
    z_right_1 = min(length(z_coord),curr_z_Indx_Num+1*(Ri+2*r_i)/dz);

    Q_mod(y_left_1:y_right_1,x_left_1:x_right_1,z_left_1:z_right_1) = c_star;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Make Scatter Plot of the Simulation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if t_count==1
        coverror_init = trapz(trapz(trapz(max(0,c_star).^3)))*dx*dy*dz;
        [xx,yy,zz] = meshgrid(x_min:dxx:x_max,y_min:dyy:y_max,z_min:dzz:z_max);
        temp = max(0,c_star-Q(1:dyy:y_max-y_min+1,1:dxx:x_max-x_min+1,1:dzz:z_max-z_min+1));
        tempp = temp(:);
        h = scatter3(xx(:),yy(:),zz(:),1,tempp,'linewidth',10);
        h.CDataSource = 'tempp';
        caxis([0 c_star-1])
        view([80,15])
        xlim([x_min, x_max])
        ylim([y_min, y_max])
        zlim([z_min, z_max])
        xlabel('x')
        ylabel('y')
        zlabel('z')
        alpha(0.1);
        axis equal;
    end

    temp = max(0,c_star-Q(1:dyy:y_max-y_min+1,1:dxx:x_max-x_min+1,1:dzz:z_max-z_min+1));
    tempp = temp(:);
    refreshdata(h)
    coverror(t_count) = trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
    coverror(t_count) = coverror(t_count)/coverror_init;

    %%%%%END COVERAGE CONTROL CODE HERE%%%%%
    Human(t+1,1) = (x_head-x_orig_ws)*10;
    Human(t+1,2) = (y_head-y_orig_ws)*10;
    Robot(t+1,1) = x_1*10;
    Robot(t+1,2) = y_1*10;
    z_one(t+1) = z_1;
    t = t+1;
    drawnow
end

Final_Time = Capture_List(end,end);

clear global qx qy qz qw x y z;
rosshutdown;
