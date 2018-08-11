clear
%Tuning Parameters
c_star=.5;
K_u=.07; K_v=.07; K_w=.07; K_r=.015; K_s=.015;
%New constants 5/19/16
lil_gam_1=10^-3;
lil_gam_2=10^-3;
lil_gam_3=10^-3;
lil_gam_4=10^-3;
lil_gam_5=10^-3;
Lam_0=1;
Lam_1=1;
cap_tol=0.5;
orient_tol=.1;
%New on 5/19/16
Ep_star=.01;
%Ep_star=10^6; %Code Test Value

%Number of agents
k=3;
Global_Flag=zeros(k,1);


%set up coordinates
dx=1; dy=1; dz=1; droll=1; dpitch=1; dyaw=1;
%set(gcf,'visible','off')
dt=.1;
time=0:dt:500;

%Uncommented on 5/19/16 to make video
%nFrames =length(time);

%mov = struct('cdata', [],...
             %           'colormap', []);
%Title='ThreeDeeCoverage_No_Energy_New_Der';                  

%vidObj = VideoWriter(Title);
%open(vidObj)     


t_count=0;
x_coord=[0:dx:150];
y_coord=[0:dy:150];
zee_coord=[-150:dz:0];
z_coord(1,1,:)=zee_coord;
roll_coord=[0:droll:2*pi];
pitch_coord=[0:dpitch:2*pi];
yaw_coord=[0:dyaw:2*pi];
% IC's will be a fixed position

x_i=zeros(k,length(time));
y_i=zeros(k,length(time));
z_i=zeros(k,length(time));
roll_i=zeros(k,length(time));
pitch_i=zeros(k,length(time));
yaw_i=zeros(k,length(time));

pos_data = importdata('position.txt');
pos_size = size(pos_data);

while pos_size(1)== 0
    pos_data = importdata('position.txt');
    pos_size = size(pos_data);
end

for kk=1:3
    
    x_i(kk,t_count+1)=pos_data(kk,2);
    y_i(kk,t_count+1)=pos_data(kk,3);
    z_i(kk,t_count+1)=pos_data(kk,4);
    roll_i(kk,t_count+1)=pos_data(kk,5);
    pitch_i(kk,t_count+1)=pos_data(kk,6);
    yaw_i(kk,t_count+1)=pos_data(kk,7);
    
end

fid = fopen('position.txt', 'wt');
fclose(fid);

Q=zeros(length(x_coord),length(y_coord),length(z_coord));
Q(146:151,:,:) = c_star;
Q(1:6,:,:) = c_star;
Q(:,146:151,:) = c_star;
Q(:,1:6,:) = c_star;
Q(:,:,146:151) = c_star;
Q(:,:,1:6) = c_star;

Q(43:79,78:116,:) = c_star;
Q(21:75,34:78,1:116) = c_star;
Q(114:151,56:114,1:126) = c_star;

% Q=ones(length(x_coord),length(y_coord),length(z_coord))*c_star;
% 
% Q(46:106,46:106,46:76) = 0;
% Q(46:56,46:56,1:46) = 0;
% Q(71:81,71:81,1:46) = 0;
% Q(96:106,96:106,96:106) = 0;
%TEMP ICS
%roll_i(t_count+1)=0;
%pitch_i(t_count+1)=0;
%yaw_i(t_count+1)=0;
%x_i(t_count+1)=5;
%y_i(t_count+1)=5;
%z_i(t_count+1)=-15;

%Size of Sensor Footprint
Ri=10;
alpha=(1/2)*120*pi/180;
update_count=0;


%Change on 5/11/16


h=ones(k,1);

%%%%New on 5/19/16
coverror=zeros(1,length(time));
coverror(1)=1;
ui_save=zeros(k,length(time));
vi_save=zeros(k,length(time));
wi_save=zeros(k,length(time));
ri_save=zeros(k,length(time));
si_save=zeros(k,length(time));
%figure('units','normalized','outerposition',[0 0 1 1])
t=0;
cap_t=t*ones(k,1);


ui=zeros(k,1);
vi=zeros(k,1);
wi=zeros(k,1);
ri=zeros(k,1);
si=zeros(k,1);
S_i=zeros(length(x_coord),length(y_coord),length(z_coord),k);

while coverror(h)>0.05;
   
tic
    t_count=t_count+1;
   
    for kk=1:k
        curr_x_Indx_Num(kk)=int64(((1/dx)*(x_i(kk,t_count)+dx)));
        curr_y_Indx_Num(kk)=int64(((1/dy)*(y_i(kk,t_count)+dy)));
        curr_z_Indx_Num(kk)=int64(((1/dz)*(max(abs(z_coord))+(z_i(kk,t_count)+dz))));
        x_left(kk)=max(1,curr_x_Indx_Num(kk)-1*Ri/dx);
        x_right(kk)=min(length(x_coord),curr_x_Indx_Num(kk)+1*Ri/dx);
        y_left(kk)=max(1,curr_y_Indx_Num(kk)-1*Ri/dy);
        y_right(kk)=min(length(y_coord),curr_y_Indx_Num(kk)+1*Ri/dy);
        z_left(kk)=max(1,curr_z_Indx_Num(kk)-1*Ri/dz);
        z_right(kk)=min(length(z_coord),curr_z_Indx_Num(kk)+1*Ri/dz);
    end
    Q_copy=Q;
    Q_mod1=Q;
    Q_mod1(x_left(2):x_right(2),y_left(2):y_right(2),z_left(2):z_right(2))=c_star;
    Q_mod1(x_left(3):x_right(3),y_left(3):y_right(3),z_left(3):z_right(3))=c_star;
    Q_mod2=Q;
    Q_mod2(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))=c_star;
    Q_mod2(x_left(3):x_right(3),y_left(3):y_right(3),z_left(3):z_right(3))=c_star;
    Q_mod3=Q;
    Q_mod3(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))=c_star;
    Q_mod3(x_left(2):x_right(2),y_left(2):y_right(2),z_left(2):z_right(2))=c_star;
    Q_copy_1=Q_mod1(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1));
    Q_copy_2=Q_mod2(x_left(2):x_right(2),y_left(2):y_right(2),z_left(2):z_right(2));
    Q_copy_3=Q_mod3(x_left(3):x_right(3),y_left(3):y_right(3),z_left(3):z_right(3));
    Si_1=zeros(x_right(1)-x_left(1)+1,y_right(1)-y_left(1)+1,z_right(1)-z_left(1)+1);
    Si_2=zeros(x_right(2)-x_left(2)+1,y_right(2)-y_left(2)+1,z_right(2)-z_left(2)+1);
    Si_3=zeros(x_right(3)-x_left(3)+1,y_right(3)-y_left(3)+1,z_right(3)-z_left(3)+1);
    
    parfor kk=1:k
    
        if kk==1;
            if Global_Flag(kk)==0;
           
            [command_1,Si_1(:,:,:,kk)]=Local_Cov2(Q_copy_1,[x_i(kk,t_count),y_i(kk,t_count),z_i(kk,t_count),pitch_i(kk,t_count),yaw_i(kk,t_count),roll_i(kk,t_count)],x_coord,y_coord,z_coord,dx,dy,dz,K_u,K_v,K_w,K_r,K_s,Ri,alpha,c_star);
            
            ui(kk)=command_1(1);
            vi(kk)=command_1(2);
            wi(kk)=command_1(3);
            qi(kk)=command_1(4);
            ri(kk)=command_1(5);
            si(kk)=command_1(6);
            
            else
            [command_1,Si_1(:,:,:,kk),orient_norm(kk)]=Global_Cov([x_i(1,t_count),y_i(1,t_count),z_i(1,t_count),pitch_i(1,t_count),yaw_i(1,t_count),roll_i(1,t_count)],[x_i(2,t_count),y_i(2,t_count),z_i(2,t_count),pitch_i(2,t_count),yaw_i(2,t_count)],[x_i(3,t_count),y_i(3,t_count),z_i(3,t_count),pitch_i(3,t_count),yaw_i(3,t_count)],x_dest(kk),y_dest(kk),z_dest(kk),pitch_dest(kk),yaw_dest(kk),x_coord,y_coord,z_coord,dx,dy,dz,Ri,alpha)
            ui(kk)=command_1(1);
            vi(kk)=command_1(2);
            wi(kk)=command_1(3);
            qi(kk)=command_1(4);
            ri(kk)=command_1(5);
            si(kk)=command_1(6);  
                
            end
        end
 
        if kk==2;
            if Global_Flag(kk)==0;
            [command_2,Si_2(:,:,:,kk)]=Local_Cov2(Q_copy_2,[x_i(2,t_count),y_i(2,t_count),z_i(2,t_count),pitch_i(2,t_count),yaw_i(2,t_count),roll_i(2,t_count)],x_coord,y_coord,z_coord,dx,dy,dz,K_u,K_v,K_w,K_r,K_s,Ri,alpha,c_star);
            ui(kk)=command_2(1);
            vi(kk)=command_2(2);
            wi(kk)=command_2(3);
            qi(kk)=command_2(4);
            ri(kk)=command_2(5);
            si(kk)=command_2(6);
            
            else
            [command_2,Si_2(:,:,:,kk),orient_norm(kk)]=Global_Cov([x_i(2,t_count),y_i(2,t_count),z_i(2,t_count),pitch_i(2,t_count),yaw_i(2,t_count),roll_i(2,t_count)],[x_i(1,t_count),y_i(1,t_count),z_i(1,t_count),pitch_i(1,t_count),yaw_i(1,t_count)],[x_i(3,t_count),y_i(3,t_count),z_i(3,t_count),pitch_i(3,t_count),yaw_i(3,t_count)],x_dest(kk),y_dest(kk),z_dest(kk),pitch_dest(kk),yaw_dest(kk),x_coord,y_coord,z_coord,dx,dy,dz,Ri,alpha)
            ui(kk)=command_2(1);
            vi(kk)=command_2(2);
            wi(kk)=command_2(3);
            qi(kk)=command_2(4);
            ri(kk)=command_2(5);
            si(kk)=command_2(6);    
                
            end
        end
        
        if kk==3;
            if Global_Flag(kk)==0;
            [command_3,Si_3(:,:,:,kk)]=Local_Cov2(Q_copy_3,[x_i(3,t_count),y_i(3,t_count),z_i(3,t_count),pitch_i(3,t_count),yaw_i(3,t_count),roll_i(3,t_count)],x_coord,y_coord,z_coord,dx,dy,dz,K_u,K_v,K_w,K_r,K_s,Ri,alpha,c_star);
            ui(kk)=command_3(1);
            vi(kk)=command_3(2);
            wi(kk)=command_3(3);
            qi(kk)=command_3(4);
            ri(kk)=command_3(5);
            si(kk)=command_3(6);
            
            else
            [command_3,Si_3(:,:,:,kk),orient_norm(kk)]=Global_Cov([x_i(3,t_count),y_i(3,t_count),z_i(3,t_count),pitch_i(3,t_count),yaw_i(3,t_count),roll_i(3,t_count)],[x_i(1,t_count),y_i(1,t_count),z_i(1,t_count),pitch_i(1,t_count),yaw_i(1,t_count)],[x_i(2,t_count),y_i(2,t_count),z_i(2,t_count),pitch_i(2,t_count),yaw_i(2,t_count)],x_dest(kk),y_dest(kk),z_dest(kk),pitch_dest(kk),yaw_dest(kk),x_coord,y_coord,z_coord,dx,dy,dz,Ri,alpha)
            ui(kk)=command_3(1);
            vi(kk)=command_3(2);
            wi(kk)=command_3(3);
            qi(kk)=command_3(4);
            ri(kk)=command_3(5);
            si(kk)=command_3(6);  
                
            end
        end
    end
    
%%%%Okay Now only update the relevant parts of the coverage Map  

 Q(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))=Q(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))+Si_1*dt;
 Q(x_left(2):x_right(2),y_left(2):y_right(2),z_left(2):z_right(2))=Q(x_left(2):x_right(2),y_left(2):y_right(2),z_left(2):z_right(2))+Si_2(:,:,:,2)*dt;
 Q(x_left(3):x_right(3),y_left(3):y_right(3),z_left(3):z_right(3))=Q(x_left(3):x_right(3),y_left(3):y_right(3),z_left(3):z_right(3))+Si_3(:,:,:,3)*dt;
toc
 
%%%%%%New Code from 5/19/16 Implement a switched law here for a Proportional controller
%for waypoint redirection. NOTE THAT qi,ri,si have changed throughout the
%code. I Have corrected them to be the same as in the paper. Also look
%specifically at the rolldot on line 461. Now includes qi

for kk=1:3
    
%%%%%%%%%%%%%%%%%%%%%%%%%
% xdot=cos(pitch_i(kk,t_count))*cos(yaw_i(kk,t_count))*ui(kk)+(sin(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*cos(yaw_i(kk,t_count))-cos(roll_i(kk,t_count))*sin(yaw_i(kk,t_count)))*vi(kk)+(cos(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*cos(yaw_i(kk,t_count))+sin(roll_i(kk,t_count))*sin(yaw_i(kk,t_count)))*wi(kk);
% ydot=cos(pitch_i(kk,t_count))*sin(yaw_i(kk,t_count))*ui(kk)+(sin(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*sin(yaw_i(kk,t_count))+cos(roll_i(kk,t_count))*cos(yaw_i(kk,t_count)))*vi(kk)+(cos(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*sin(yaw_i(kk,t_count))-sin(roll_i(kk,t_count))*cos(yaw_i(kk,t_count)))*wi(kk);
% zdot=-sin(pitch_i(kk,t_count))*ui(kk)+(sin(roll_i(kk,t_count))*cos(pitch_i(kk,t_count)))*vi(kk)+(cos(roll_i(kk,t_count))*cos(pitch_i(kk,t_count)))*wi(kk);
% %New term on Rolldot. Also I have fixed the ri and si names throughout the code as they were inconsistent with the paper 5/19/16 
% rolldot=qi(kk)+sin(roll_i(kk,t_count))*tan(pitch_i(kk,t_count))*ri(kk)+cos(roll_i(kk,t_count))*tan(pitch_i(kk,t_count))*si(kk);
% pitchdot=cos(roll_i(kk,t_count))*ri(kk)-sin(roll_i(kk,t_count))*si(kk);
% yawdot=sin(roll_i(kk,t_count))*sec(pitch_i(kk,t_count))*ri(kk)+cos(roll_i(kk,t_count))*sec(pitch_i(kk,t_count))*si(kk);
% 
% x_i(kk,t_count+1)=x_i(kk,t_count)+xdot*dt;
% y_i(kk,t_count+1)=y_i(kk,t_count)+ydot*dt;
% z_i(kk,t_count+1)=z_i(kk,t_count)+zdot*dt;
% roll_i(kk,t_count+1)=roll_i(kk,t_count)+rolldot*dt;
% pitch_i(kk,t_count+1)=pitch_i(kk,t_count)+pitchdot*dt;
% yaw_i(kk,t_count+1)=yaw_i(kk,t_count)+yawdot*dt;

%%%%%New Code from 5/19/16
ui_save(kk,h(kk))=ui(kk);
vi_save(kk,h(kk))=vi(kk);
wi_save(kk,h(kk))=wi(kk);
ri_save(kk,h(kk))=ri(kk);
si_save(kk,h(kk))=si(kk);

if Global_Flag(kk)==1 && sqrt((x_i(kk,t_count+1)-x_dest(kk))^2+(y_i(kk,t_count+1)-y_dest(kk))^2+(z_i(kk,t_count+1)-z_dest(kk))^2)<cap_tol && orient_norm(kk)<orient_tol;
    Global_Flag(kk)=1;
    fprintf('Captured new Orientation at waypoint for Agent:\n')
    kk;
    cap_t(kk)=t;
    j_tilde(kk)=0;
end

if h(kk)==1
    coverror_init=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
end
 
coverror(h(kk))=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
coverror(h(kk))=coverror(h(kk))/coverror_init;
    

  h(kk)=h(kk)+1;
  %Only to make while loop work. This gets overwritten each cycle. 
  coverror(h(kk))=coverror(h(kk)-1); 
    
    
   
%Global Coverage Code!!! All new here 5/19/16 basically until end of code
%if t>(1+cap_t(kk))
%if abs(coverror(h(kk)-10)-coverror(h(kk)-1))<Ep_star && Global_Flag(kk)==0
% NEW CODE 7/12/16. Let's use a velocity command threshold instead of a
% coverage threshold. That way global coverage can occur for individual
% agents not just all of them at once
    if exist('j_tilde')==1
        if max(size(j_tilde))==3
    if sum(j_tilde==[0,0,0])==3
        clear j_tilde;
    end
        end
    end
     if exist('j_tilde')==1   
     if max(size(j_tilde))==2
    if sum(j_tilde==[0,0])==2
        clear j_tilde;
    end
     end
     end
     if exist('j_tilde')==1
         if max(size(j_tilde))==1
    if sum(j_tilde==[0])==1
        clear j_tilde;
    end
        end
    end


if sqrt(ui(kk)^2+vi(kk)^2+wi(kk)^2+ri(kk)^2+si(kk)^2)<Ep_star && Global_Flag(kk)==0

fprintf('Entering Global Coverage Mode\n')    



x_glo_dex(kk)=1;
x_glo_cen(kk,x_glo_dex(kk))=floor((1/2)*sqrt(2)*Ri/dx);
y_glo_dex(kk)=1;
y_glo_cen(kk,y_glo_dex(kk))=floor((1/2)*sqrt(2)*Ri/dy);
z_glo_dex(kk)=1;
z_glo_cen(kk,z_glo_dex(kk))=floor((1/2)*sqrt(2)*Ri/dz);
glo_dex(kk)=1;
while x_glo_dex(kk)<length(Q(1,:,:))
    y_glo_dex(kk)=1;
    while y_glo_dex(kk)<length(Q(:,1,:)) 
        z_glo_dex(kk)=1;
        while z_glo_dex(kk)<length(Q(:,:,1))
          
            x_glo_left(kk)=max(1,x_glo_cen(kk,x_glo_dex(kk))-floor((1/2)*sqrt(2)*Ri/dx));
            x_glo_right(kk)=min(length(Q(:,1,1)),x_glo_cen(kk,x_glo_dex(kk))+floor((1/2)*sqrt(2)*Ri/dx));
            y_glo_left(kk)=max(1,y_glo_cen(kk,y_glo_dex(kk))-floor((1/2)*sqrt(2)*Ri/dy));
            y_glo_right(kk)=min(length(Q(1,:,1)),y_glo_cen(kk,y_glo_dex(kk))+floor((1/2)*sqrt(2)*Ri/dy));
            z_glo_left(kk)=max(1,z_glo_cen(kk,z_glo_dex(kk))-floor((1/2)*sqrt(2)*Ri/dz));
            z_glo_right(kk)=min(length(Q(1,1,:)),z_glo_cen(kk,z_glo_dex(kk))+floor((1/2)*sqrt(2)*Ri/dz));
            %Calculate the C_hat expression. Coverage in each subdomain
            %NEW CODE CHANGE ON 7/12/16
            if kk==1
            C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)= trapz(trapz(trapz(min(c_star,Q_mod1(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx ...
                                                  +dirac( ...
                                                        c_star*(x_glo_right(kk)-x_glo_left(kk))*dx*(y_glo_right(kk)-y_glo_left(kk))*dy*(z_glo_right(kk)-z_glo_left(kk))*dz ...
                                                        -trapz(trapz(trapz(min(c_star,Q_mod1(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx);
            end
            if kk==2
            C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)= trapz(trapz(trapz(min(c_star,Q_mod2(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx ...
                                                  +dirac( ...
                                                        c_star*(x_glo_right(kk)-x_glo_left(kk))*dx*(y_glo_right(kk)-y_glo_left(kk))*dy*(z_glo_right(kk)-z_glo_left(kk))*dz ...
                                                        -trapz(trapz(trapz(min(c_star,Q_mod2(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx);
            end
            if kk==3
                C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)= trapz(trapz(trapz(min(c_star,Q_mod3(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx ...
                                                  +dirac( ...
                                                        c_star*(x_glo_right(kk)-x_glo_left(kk))*dx*(y_glo_right(kk)-y_glo_left(kk))*dy*(z_glo_right(kk)-z_glo_left(kk))*dz ...
                                                        -trapz(trapz(trapz(min(c_star,Q_mod3(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx);    
            end
            %Rearrange information into a more logical matrix here
            C_hat_concat(1,glo_dex(kk),kk)=C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk);
            C_hat_concat(2,glo_dex(kk),kk)=x_glo_cen(kk,x_glo_dex(kk));
            C_hat_concat(3,glo_dex(kk),kk)=y_glo_cen(kk,y_glo_dex(kk));
            C_hat_concat(4,glo_dex(kk),kk)=z_glo_cen(kk,z_glo_dex(kk));
         
             
                  
            
               if exist('j_tilde')==1

                   if squeeze(max((C_hat_concat(2,nonzeros(j_tilde(:)),:)==C_hat_concat(2,glo_dex(kk),kk)) +  (C_hat_concat(3,nonzeros(j_tilde(:)),:)==C_hat_concat(3,glo_dex(kk),kk)) +  (C_hat_concat(4,nonzeros(j_tilde(:)),:)==C_hat_concat(4,glo_dex(kk),kk))))==[3;3;3]
                C_hat_concat(1,glo_dex(kk),kk)=inf;
                    end
                    end

            glo_dex(kk)=glo_dex(kk)+1;
            if z_glo_right(kk)<length(Q(1,1,:))
            z_glo_cen(kk,z_glo_dex(kk)+1)=min(length(Q(1,1,:))-floor((1/2)*sqrt(2)*Ri/dz),z_glo_cen(kk,z_glo_dex(kk))+2*floor((1/2)*sqrt(2)*Ri/dz));
            z_glo_dex(kk)=z_glo_dex(kk)+1;
            else
                break
            end
        end
        if y_glo_right(kk)<length(Q(1,:,1))
            y_glo_cen(kk,y_glo_dex(kk)+1)=min(length(Q(1,:,1))-floor((1/2)*sqrt(2)*Ri/dy),y_glo_cen(kk,y_glo_dex(kk))+2*floor((1/2)*sqrt(2)*Ri/dy));
            y_glo_dex(kk)=y_glo_dex(kk)+1;
            else
                break
        end
    end
    
        if x_glo_right(kk)<length(Q(:,1,1))
            x_glo_cen(kk,x_glo_dex(kk)+1)=min(length(Q(:,1,1))-floor((1/2)*sqrt(2)*Ri/dx),x_glo_cen(kk,x_glo_dex(kk))+2*floor((1/2)*sqrt(2)*Ri/dx));
            x_glo_dex(kk)=x_glo_dex(kk)+1;
            else
                break
        end
end

big_gam(kk)=trapz(lil_gam_1*abs(ui_save(kk,:))+lil_gam_2*abs(vi_save(kk,:))+lil_gam_3*abs(wi_save(kk,:))+lil_gam_4*abs(ri_save(kk,:))+lil_gam_5*abs(si_save(kk,:)))*dt;



[trash(kk),j_tilde(kk)]=min((Lam_0+big_gam(kk))*sqrt((repmat(x_i(kk,t_count+1),1,length(C_hat_concat(2,:,kk)))-x_coord(C_hat_concat(2,:,kk))).^2+(repmat(y_i(kk,t_count+1),1,length(C_hat_concat(3,:,kk)))-y_coord(C_hat_concat(3,:,kk))).^2+(repmat(z_i(kk,t_count+1),1,length(C_hat_concat(4,:,kk)))-zee_coord(C_hat_concat(4,:,kk))).^2)+Lam_1*C_hat_concat(1,:,kk));
x_dest(kk)=x_coord(C_hat_concat(2,j_tilde(kk),kk));
y_dest(kk)=y_coord(C_hat_concat(3,j_tilde(kk),kk));
z_dest(kk)=z_coord(C_hat_concat(4,j_tilde(kk),kk));
orient_glo_dex(kk)=1;

for yaw_test=0:alpha:2*pi %Need to avoid gimbal lock
     pitch_test=0;


        curr_x_Indx_Num(kk)=int64(((1/dx)*(x_dest(kk)+dx)));
        curr_y_Indx_Num(kk)=int64(((1/dy)*(y_dest(kk)+dy)));
        curr_z_Indx_Num(kk)=int64(((1/dz)*(max(abs(z_coord))+(z_dest(kk)+dz))));

x_left(kk)=max(1,curr_x_Indx_Num(kk)-1.5*Ri/dx);
x_right(kk)=min(length(x_coord),curr_x_Indx_Num(kk)+1.5*Ri/dx);
y_left(kk)=max(1,curr_y_Indx_Num(kk)-1.5*Ri/dy);
y_right(kk)=min(length(y_coord),curr_y_Indx_Num(kk)+1.5*Ri/dy);
z_left(kk)=max(1,curr_z_Indx_Num(kk)-1.5*Ri/dz);
z_right(kk)=min(length(z_coord),curr_z_Indx_Num(kk)+1.5*Ri/dz);


x_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk) = repmat(x_coord(x_left(kk):x_right(kk))',1,length(y_coord(y_left(kk):y_right(kk))),length(z_coord(z_left(kk):z_right(kk))));
y_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk) = repmat(y_coord(y_left(kk):y_right(kk)),length(x_coord(x_left(kk):x_right(kk))),1,length(z_coord(z_left(kk):z_right(kk))));
z_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk) = repmat(z_coord(z_left(kk):z_right(kk)),length(x_coord(x_left(kk):x_right(kk))),length(y_coord(y_left(kk):y_right(kk))),1);
phi(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))...
    =acos((((x_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-x_dest(kk)).*cos(yaw_test).*cos(pitch_test)...
    +(y_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-y_dest(kk)).*sin(yaw_test).*cos(pitch_test)-...
    (z_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-z_dest(kk)).*sin(pitch_test))...
    ./sqrt((x_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-x_dest(kk)).^2....
    +(y_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-y_dest(kk)).^2....
    +(z_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-z_dest(kk)).^2)));

c1(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=Ri^2-(repmat(x_coord(x_left(kk):x_right(kk))',1,length(y_coord(y_left(kk):y_right(kk))),length(z_coord(z_left(kk):z_right(kk))))-x_dest(kk)).^2-(repmat(y_coord(y_left(kk):y_right(kk)),length(x_coord(x_left(kk):x_right(kk))),1,length(z_coord(z_left(kk):z_right(kk))))-y_dest(kk)).^2-(repmat(z_coord(z_left(kk):z_right(kk)),length(x_coord(x_left(kk):x_right(kk))),length(y_coord(y_left(kk):y_right(kk))),1)-z_dest(kk)).^2;
c2(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=alpha-phi(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk));
c3(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=alpha+phi(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk));
B(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=(1/max(0,c1(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))))+(1/max(0,c2(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))))+(1/max(0,c3(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))));
Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)=1./B(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk));
Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)=Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)./max(max(max(Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk))));

Arg_Omega(kk,1,orient_glo_dex(kk))=trapz(trapz(trapz((3*(c_star-Q(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))).^2).*Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk),3).*dz,2).*dy,1).*dx;
Arg_Omega(kk,2,orient_glo_dex(kk))=yaw_test;
Arg_Omega(kk,3,orient_glo_dex(kk))=pitch_test;
    
orient_glo_dex(kk)=orient_glo_dex(kk)+1;


end
[trash(kk),Arg_dex(kk)]=max(Arg_Omega(kk,1,:));
        yaw_dest(kk)=Arg_Omega(kk,2,Arg_dex(kk));
pitch_dest(kk)=0;

%Test Values
%x_dest=20;
%y_dest=20;
%z_dest=-10;
%pitch_dest=pi/4;
%yaw_dest=pi;


Global_Flag(kk)=1;
    end
end


%[trash,dest2]=max(e(:));
%clear trash




% end
%end
% end
% PQ(:,:,:)=(((Q-Q_copy)*dt));
% 
% 
% test_count_1=0;
% clear Test_shape_1
% 
% for xx=1:1:length(x_coord)
%         for yy=1:1:length(y_coord)
%             for zz=1:1:length(z_coord)
% %%%%%DELETE BELOW AND THEN GO UP AND DELETE test_count initialization
% if PQ(xx,yy,zz)>0
% test_count_1=test_count_1+1;
% Test_shape_1(test_count_1,:)=[x_coord(xx) y_coord(yy) z_coord(zz) PQ(xx,yy,zz)];
% end
%             end
%         end
% end
% 
% 
% %subplot(1,2,2)
% scatter3(Test_shape_1(:,1),Test_shape_1(:,2),Test_shape_1(:,3),5,Test_shape_1(:,4));
% hold on;
% axis([min(x_coord),max(x_coord),min(y_coord),max(y_coord),min(z_coord),max(z_coord)])
% 
% %coverror=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
%  %   title(['Time = ', num2str((h-1)*dt), ' sec. Coverage Error= ',num2str(coverror)])
% 
% title(['Cov. Time = ', num2str((h(kk)-2)*dt), ' sec. Coverage Error= ',num2str(coverror(h(kk)-1))])
%   if h(kk)>2
%     delete(boxy1)
%     delete(boxy2)
%     delete(boxy3)
%   end
%     if Global_Flag(1)==0
%     string1=['Agent 1 Local Mode'];
%     else
%     string1=['Agent 1 Global Mode'];
%     end
%         if Global_Flag(2)==0
%     string2=['Agent 2 Local Mode'];
%     else
%     string2=['Agent 2 Global Mode'];
%     end
%         if Global_Flag(3)==0
%     string3=['Agent 3 Local Mode'];
%     else
%     string3=['Agent 3 Global Mode'];
%     end
%     
%     boxy1=annotation('textbox',[.1 .83 .1 .1],'string',string1,'EdgeColor','none');
%     boxy2=annotation('textbox',[.1 .80 .1 .1],'string',string2,'EdgeColor','none');
%     boxy3=annotation('textbox',[.1 .77 .1 .1],'string',string3,'EdgeColor','none');
%  
% 
% hold off;
% 
% 
% 
%  drawnow update
% 
%   mov= getframe(gcf);
%     
%     writeVideo(vidObj,mov)
%  
%      clear mov
%     mov = struct('cdata', [],...
%                        'colormap', []);
%  
% t=t+dt;

  
coverr=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
coverr=coverr/coverror_init;


speed_data = [1 coverr; ui(1) vi(1); wi(1) si(1); ui(2) vi(2);wi(2) si(2); ui(3) vi(3); wi(3) si(3)];
save speed.txt -ascii speed_data
save data.txt -ascii speed_data

pos_data = importdata('position.txt');
pos_size = size(pos_data);

while pos_size(1)== 0
    pos_data = importdata('position.txt');
    pos_size = size(pos_data);
end

for kk=1:3
    
    x_i(kk,t_count+1)=pos_data(kk,2);
    y_i(kk,t_count+1)=pos_data(kk,3);
    z_i(kk,t_count+1)=pos_data(kk,4);
    roll_i(kk,t_count+1)=roll_coord(11);
    pitch_i(kk,t_count+1)=pitch_coord(11);
    yaw_i(kk,t_count+1)=yaw_coord(11);
    
end

fid = fopen('position.txt', 'wt');
fclose(fid);



end



% Create AVI file.
% close(vidObj);
