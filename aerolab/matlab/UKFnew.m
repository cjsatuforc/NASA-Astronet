function [x,y,z,roll,pitch,yaw] = UKFnew(x,y,z,roll,pitch,yaw,ax,ay,az,wx,wy,wz,dt,first,...
    pos_cov,accel_cov,angular_cov,orient_cov,k)
% Unscented Kalman Filter

% Unpack covariances: 

pos_cov = [pos_cov(1) pos_cov(2) pos_cov(3); pos_cov(4) pos_cov(5) pos_cov(6); pos_cov(7) pos_cov(8) pos_cov(9)];
%Will is gonna change the pos cov manually to 200 on altimeter
if pos_cov(1,1)~=0
    pos_cov(3,3)=1000000000;
end
    

accel_cov = [accel_cov(1) accel_cov(2) accel_cov(3); accel_cov(4) accel_cov(5) accel_cov(6); accel_cov(7) accel_cov(8) accel_cov(9)];
angular_cov = [angular_cov(1) angular_cov(2) angular_cov(3); angular_cov(4) angular_cov(5) angular_cov(6); angular_cov(7) angular_cov(8) angular_cov(9)];
orient_cov = [orient_cov(1) orient_cov(2) orient_cov(3); orient_cov(4) orient_cov(5) orient_cov(6); orient_cov(7) orient_cov(8) orient_cov(9)];

vel_cov = eye(3)*1e-3; % Not provided by IMU, but needed for UKF; value is purely arbitrary and should be adjusted as necessary

% Rotate measurements and covariances:

global rot

rot = eul2rotm([yaw pitch roll]);
accel_grav=rot'*[0;0;-9.80]; %at our location
%account for bias these numbers from yellow agent
ax=ax+accel_grav(1)+0.1016;
ay=ay+accel_grav(2)-0.0685;
az=az+accel_grav(3)+0.3614-.05;

accel = rot*[ax; ay; az];
ax = accel(1); ay = accel(2); az = accel(3);
az;
omega = rot*[wx; wy; wz];
wx = omega(1); wy = omega(2); wz = omega(3);
%Add Position Covariance Rotation
accel_cov = rot*accel_cov*rot';
angular_cov = rot*angular_cov*rot';
orient_cov = rot*orient_cov*rot';
vel_cov = rot*vel_cov*rot';

% Initialize:

global controls deltaT Q R P s

n = 12;      %number of states

if first
    
    vx = 0;
    vy = 0;
    vz = 0;
    ax = 0;
    ay = 0;
    az = 0;
    
    Q{k} = [eye(12)*.1]; % covariance of process, can be tuned
    R{k} = [pos_cov zeros(3,6);...
        zeros(3,3) angular_cov,zeros(3,3);...
        zeros(3,6),accel_cov]; % covariance of measurement
    s{k} = [x; y; 0; roll; pitch; yaw; vx; vy; vz;ax;ay;az]; % initial state z is 50 for the IMU test with no Barometer
    P{k} = eye(n); % initial state covariance
    
end

    R{k} = [pos_cov zeros(3,6);...
        zeros(3,3) angular_cov,zeros(3,3);...
        zeros(3,6),accel_cov];

meas = [x; y; z; roll; pitch; yaw; ax; ay; az];
f = @F;  % nonlinear state equations
h = @H;  % measurement equation

controls = [ax ay az wx wy wz];
s{2}
deltaT = dt;

% UKF:

[s{k}, P{k}] = ukf(f,s{k},P{k},h,meas,Q{k},R{k});

output = s{k};
x = output(1); y = output(2); z = output(3);
roll = output(4); pitch = output(5); yaw = output(6);

end

function X = F(state)

global controls deltaT rot

% Unpack variables:

vx = state(7); vy = state(8); vz = state(9);
phi = state(4); theta = state(5); psi = state(6);

ax = controls(1); ay = controls(2); az = controls(3);
wx = controls(4); wy = controls(5); wz = controls(6);

% Rotate velocities into body-fixed frame:

vBody = rot'*[vx; vy; vz];
u = vBody(1); v = vBody(2); w = vBody(3);
wBody = rot'*[wx; wy; wz];
q = wBody(1); r = wBody(2); s = wBody(3);

% Calculate rates of change:

R1 = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
    cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
    -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
R2 = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

EulAdd_1=(R2(1,1)*q+R2(1,2)*r+R2(1,3)*s)*deltaT;
EulAdd_2=(R2(2,1)*q+R2(2,2)*r+R2(2,3)*s)*deltaT;
EulAdd_3=(R2(3,1)*q+R2(3,2)*r+R2(3,3)*s)*deltaT;
LinAdd_1=(R1(1,1)*u+R1(1,2)*v+R1(1,3)*w)*deltaT;
LinAdd_2=(R1(2,1)*u+R1(2,2)*v+R1(2,3)*w)*deltaT;
LinAdd_3=(R1(3,1)*u+R1(3,2)*v+R1(3,3)*w)*deltaT;

%13 14 and 15 are accelerometer biases
%This doesn't work to well. You should go back and
%Remove the states 13-16 and uncomment
%the bias stuff at the beginning. 
X = [state(1)+state(7)*deltaT;state(2)+state(8)*deltaT;state(3)+state(9)*deltaT; state(4)+EulAdd_1;state(5)+EulAdd_2;state(6)+EulAdd_3;state(7)+state(10)*deltaT; state(8)+state(11)*deltaT; state(9)+state(12)*deltaT;state(10);state(11);state(12)];

end

function meas = H(state)
phi = state(4); theta = state(5); psi = state(6);
R1 = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
    cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
    -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];

meas = [state(1:6);state(10);state(11);state(12)];

end
