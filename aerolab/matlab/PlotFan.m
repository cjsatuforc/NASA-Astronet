function h = PlotFan(ax,nSeg,pan) 

ang = linspace( -pan/2, pan/2, nSeg+1)';
x = zeros(2,nSeg+1);
y = x;

x(2,:) = sind(ang) * 75;
y(2,:) = cosd(ang) * 75;

h = line(x,y,'color',[ 0 0 1], 'parent', ax, 'handleVisibility','off');
