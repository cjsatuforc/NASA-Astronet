clear
load('GPS_Walk_1-19_final.mat')
%set end to be less than or equal to t_count-1
endd=t_count-1;


% replace with an image of your choice
img = imread('Selection_002_Scaled.png');
 
% set the range of the axes
% The image will be stretched to this.
min_x = -83.712;
max_x = -83.7114;
min_y = 42.2928;
max_y = 42.2935;
 
% make data to plot - just a line.
x = ((x_i_gpswalk(2,1:endd)-100)*(30/200)/111078.9538)+42.293233218565383;
y = -(((y_i_gpswalk(2,1:endd)-100)*(30/200)/82469.093)+83.711667016455664);

% Flip the image upside down before showing it
imagesc([min_x max_x], [min_y max_y], flipud(img));
 
% NOTE: if your image is RGB, you should use flipdim(img, 1) instead of flipud.
 
hold all;
plot(y,x,'linewidth',1.5);
plot(long_save(1:endd),lat_save(1:endd))
% set the y-axis back to normal.
set(gca,'ydir','normal');
hold off