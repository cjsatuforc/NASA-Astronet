%------------------------ Define viconCallback -----------------------%
function viconCallback_2(src,msg)
    global qx_2 qy_2 qz_2 qw_2 x_2 y_2 z_2;
    qx_2 = msg.Transform.Rotation.X;
    qy_2 = msg.Transform.Rotation.Y;
    qz_2 = msg.Transform.Rotation.Z;
    qw_2 = msg.Transform.Rotation.W;
    
    x_2 = msg.Transform.Translation.X;
    y_2 = msg.Transform.Translation.Y;
    z_2 = msg.Transform.Translation.Z;
    
    %True/Zhipeng Transformations
    %x_2 = (x_2+1.42)*200/1.8;
    %y_2 = (y_2+0.12)*200/1.8;
    %z_2 = (z_2-0.16)*100/1.5-100;
    
    %New Will Transformations (teenier)
    
    %x_2 = (x_2+1.27)*200/1.5;
    %y_2 = (y_2-.03)*200/1.5;
    %z_2 = (z_2-0.2)*100/1.5-100;
    
    %Newer Will (teenierer)
    % x_2 = (x_2+1.22)*200/1.4;
    % y_2 = (y_2-.08)*200/1.4;
    % z_2 = (z_2-0.2)*100/1.5-100;
   % x_2 = (x_2+1.52)*200/2;
   % y_2 = (y_2+0.22)*200/2;
   % z_2 = (z_2-0.16)*100/2-100;
    
    %Larger Ones
    x_2 = (x_2+1.775)*200/2.55;
    y_2 = (y_2+0.345)*200/2.55;
    z_2 = (z_2-0.2)*100/2.3-100;
end

        
%---------------------------- END ------------------------------------%