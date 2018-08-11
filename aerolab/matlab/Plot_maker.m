load('flight_3_28_17_f1.mat')

 nFrames =length(time);
 
 mov = struct('cdata', [],...
                         'colormap', []);
 Title='MAR_28';                  
 
 vidObj = VideoWriter(Title);
 vidObj.FrameRate=57;
 open(vidObj)


Q=zeros(length(x_coord),length(y_coord),length(z_coord));
figure
colormap winter
for t_count=1:1:8642
    
    
for kk=1:k    
curr_x_Indx_Num=int64(((1/dx)*(x_i(kk,t_count)+dx)));
curr_y_Indx_Num=int64(((1/dy)*(y_i(kk,t_count)+dy)));
%This one is different. Note that z is defined down and the array runs
%backwards. Look at the z_coord compared to x_coord if you don't understand
curr_z_Indx_Num=int64(((1/dz)*(max(abs(z_coord))+(z_i(kk,t_count)+dz))));


%THIS BLOCK HAS CHANGED 6/9/16. Note the 1's instead of 1.5's. Box is now
%2Rx2Rx2R instead of 3R etc
x_left=max(1,curr_x_Indx_Num-1*Ri/dx);
x_right=min(length(x_coord),curr_x_Indx_Num+1*Ri/dx);
y_left=max(1,curr_y_Indx_Num-1*Ri/dy);
y_right=min(length(y_coord),curr_y_Indx_Num+1*Ri/dy);
z_left=max(1,curr_z_Indx_Num-1*Ri/dz);
z_right=min(length(z_coord),curr_z_Indx_Num+1*Ri/dz);

x_mat = repmat(x_coord(x_left:x_right)',1,length(y_coord(y_left:y_right)),length(z_coord(z_left:z_right)));
y_mat = repmat(y_coord(y_left:y_right),length(x_coord(x_left:x_right)),1,length(z_coord(z_left:z_right)));
z_mat = repmat(z_coord(z_left:z_right),length(x_coord(x_left:x_right)),length(y_coord(y_left:y_right)),1);
phi...
    =acos((((x_mat-x_i(kk,t_count)).*cos(yaw_i(kk,t_count)).*cos(pitch_i(kk,t_count))...
    +(y_mat-y_i(kk,t_count)).*sin(yaw_i(kk,t_count)).*cos(pitch_i(kk,t_count))-...
    (z_mat-z_i(kk,t_count)).*sin(pitch_i(kk,t_count)))...
    ./sqrt((x_mat-x_i(kk,t_count)).^2....
    +(y_mat-y_i(kk,t_count)).^2....
    +(z_mat-z_i(kk,t_count)).^2)));

c1=Ri^2-(repmat(x_coord(x_left:x_right)',1,length(y_coord(y_left:y_right)),length(z_coord(z_left:z_right)))-x_i(kk,t_count)).^2-(repmat(y_coord(y_left:y_right),length(x_coord(x_left:x_right)),1,length(z_coord(z_left:z_right)))-y_i(kk,t_count)).^2-(repmat(z_coord(z_left:z_right),length(x_coord(x_left:x_right)),length(y_coord(y_left:y_right)),1)-z_i(kk,t_count)).^2;
c2=alpha-phi;
c3=alpha+phi;
B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
Si=1./B;
Si=Si./max(max(max(Si)));

if isnan(max(Si(:)))==0
Q(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))=Q(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))+Si*dt;
end
end  
     
  test_count_1=0;
  clear Test_shape_1
  
  for xx=1:1:length(x_coord)
          for yy=1:1:length(y_coord)
              for zz=1:1:length(z_coord)
% % %%%%%DELETE BELOW AND THEN GO UP AND DELETE test_count initialization
  if Q(xx,yy,zz)>0
  test_count_1=test_count_1+1;
  Test_shape_1(test_count_1,:)=[x_coord(xx) y_coord(yy) z_coord(zz) min(Q(xx,yy,zz),c_star)];
  end
              end
          end
  end
  
  %subplot(1,2,2)
  scatter3(Test_shape_1(:,1),Test_shape_1(:,2),Test_shape_1(:,3),5,Test_shape_1(:,4));
  hold on;
  axis([min(x_coord),max(x_coord),min(y_coord),max(y_coord),min(z_coord),max(z_coord)])
  
%  coverror=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
%     %title(['Time = ', num2str((h-1)*dt), ' sec. Coverage Error= ',num2str(coverror)])
%  
%  %title(['Cov. Time = ', num2str((h(kk)-2)*dt), ' sec. Coverage Error= ',num2str(coverror(h(kk)-1))])
%  %title(['Cov. Time = ', num2str((h(kk)-2)*dt), ' sec. Coverage Error= ',num2str(coverror(1))])
% 
%title([num2str(t_count), '...',num2str(Global_Flag(1)),'...',num2str(Global_Flag(2))])
  plot3([x_i(1,t_count) x_i(1,t_count)],[y_i(1,t_count) y_i(1,t_count)],[z_i(1,t_count) z_i(1,t_count)],'or', 'MarkerSize', 5,'LineWidth',2,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor',[1 0 0])

            %Agent 2 and 3
            %   plot3([x_i(2,t_count) x_i(2,t_count)],[y_i(2,t_count) y_i(2,t_count)],[z_i(2,t_count) z_i(2,t_count)],'or', 'MarkerSize', 12,'LineWidth',2,...
 %               'MarkerEdgeColor','r',...
  %              'MarkerFaceColor',[1 0 0])
 %  plot3([x_i(3,t_count) x_i(3,t_count)],[y_i(3,t_count) y_i(3,t_count)],[z_i(3,t_count) z_i(3,t_count)],'or', 'MarkerSize', 12,'LineWidth',2,...
  %              'MarkerEdgeColor','r',...
   %             'MarkerFaceColor',[1 0 0])
   h = colorbar;
set(h, 'ylim', [0 50])
caxis([0 50])
view([231 14]) %side view
%view([45 90]) %top view
%  
  hold off;
%  
  drawnow update
 mov= getframe(gcf);
     
     writeVideo(vidObj,mov)
  
      clear mov
     mov = struct('cdata', [],...
                        'colormap', []);

t_count
end
close(vidObj)
