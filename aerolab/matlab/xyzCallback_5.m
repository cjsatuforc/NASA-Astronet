%------------------------ Define xyzCallback -----------------------%
function xyzCallback_5(src,msg)
    global qx_5 qy_5 qz_5 qw_5;
    global x_5 y_5 z_5;
%     qx_1 = msg.Transform.Rotation.X;
%     qy_1 = msg.Transform.Rotation.Y;
%     qz_1 = msg.Transform.Rotation.Z;
%     qw_1 = msg.Transform.Rotation.W;
    
    x_5 = msg.X;
    y_5 = msg.Y;
    z_5 = msg.Z;
    
    %True/Zhipeng Transformations
   % x_1 = (x_1+1.42)*200/1.8;
   % y_1 = (y_1+0.12)*200/1.8;
   % z_1 = (z_1-0.16)*100/1.5-100;
    
    %New Will Transformations (teenier)
    
    %x_1 = (x_1+1.27)*200/1.5;
    %y_1 = (y_1-.03)*200/1.5;
    %z_1 = (z_1-0.2)*100/1.5-100;
    
        %Newer Will (teenierer)
   %  x_1 = (x_1+1.22)*200/1.4;
   %  y_1 = (y_1-.08)*200/1.4;
   %  z_1 = (z_1-0.2)*100/1.5-100;
   
   %Larger Ones
    x_5 = (x_5+1.775)*200/2.55;
    y_5 = (y_5+0.345)*200/2.55;
    z_5 = (z_5-0.25)*100/2.35-100;
end
        
%---------------------------- END ------------------------------------%