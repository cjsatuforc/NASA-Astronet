%------------------------ Define viconCallback -----------------------%
function viconCallback_6(src,msg)
    global qx_6 qy_6 qz_6 qw_6 x_6 y_6 z_6;
    qx_6 = msg.Transform.Rotation.X;
    qy_6 = msg.Transform.Rotation.Y;
    qz_6 = msg.Transform.Rotation.Z;
    qw_6 = msg.Transform.Rotation.W;
    
    x_6 = msg.Transform.Translation.X;
    y_6 = msg.Transform.Translation.Y;
    z_6 = msg.Transform.Translation.Z;
    
    %True/Zhipeng Transformations
    %x = (x+1.42)*200/1.8;
    %y = (y+0.12)*200/1.8;
    %z = (z-0.06)*100/1.5-100;
    
    %New Will Transformations (teenier)
    
    x_6 = (x_6+1.27)*200/1.5;
    y_6 = (y_6-.03)*200/1.5;
    z_6 = (z_6-0.16)*100/1.5-100;
end
        
%---------------------------- END ------------------------------------%