%------------------------ Define viconCallback -----------------------%
function viconCallback_5(src,msg)
    global qx_5 qy_5 qz_5 qw_5 x_5 y_5 z_5;
    qx_5 = msg.Transform.Rotation.X;
    qy_5 = msg.Transform.Rotation.Y;
    qz_5 = msg.Transform.Rotation.Z;
    qw_5 = msg.Transform.Rotation.W;
    
    x_5 = msg.Transform.Translation.X;
    y_5 = msg.Transform.Translation.Y;
    z_5 = msg.Transform.Translation.Z;
    
    %True/Zhipeng Transformations
    %x = (x+1.42)*200/1.8;
    %y = (y+0.12)*200/1.8;
    %z = (z-0.06)*100/1.5-100;
    
    %New Will Transformations (teenier)
    
    x_5 = (x_5+1.27)*200/1.5;
    y_5 = (y_5-.03)*200/1.5;
    z_5 = (z_5-0.16)*100/1.5-100;
end
        
%---------------------------- END ------------------------------------%