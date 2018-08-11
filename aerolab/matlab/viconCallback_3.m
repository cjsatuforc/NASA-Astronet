%------------------------ Define viconCallback -----------------------%
function viconCallback_3(src,msg)
    global qx_3 qy_3 qz_3 qw_3 x_3 y_3 z_3;
    qx_3 = msg.Transform.Rotation.X;
    qy_3 = msg.Transform.Rotation.Y;
    qz_3 = msg.Transform.Rotation.Z;
    qw_3 = msg.Transform.Rotation.W;
    
    x_3 = msg.Transform.Translation.X;
    y_3 = msg.Transform.Translation.Y;
    z_3 = msg.Transform.Translation.Z;
    
    %True/Zhipeng Transformations
    %x = (x+1.42)*200/1.8;
    %y = (y+0.12)*200/1.8;
    %z = (z-0.06)*100/1.5-100;
    
    %New Will Transformations (teenier)
    
   % x_3 = (x_3+1.22)*200/1.4;
   % y_3 = (y_3-.08)*200/1.4;
   % z_3 = (z_3-0.2)*100/1.5-100;
    
        %Larger Ones
    x_3 = (x_3+1.775)*200/2.55;
    y_3 = (y_3+0.345)*200/2.55;
    z_3 = (z_3-0.2)*100/2.3-100;
end
        
%---------------------------- END ------------------------------------%