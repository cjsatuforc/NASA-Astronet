%------------------------ Define viconCallback -----------------------%
function viconCallback_4(src,msg)
    global qx_4 qy_4 qz_4 qw_4 x_4 y_4 z_4;
    qx_4 = msg.Transform.Rotation.X;
    qy_4 = msg.Transform.Rotation.Y;
    qz_4 = msg.Transform.Rotation.Z;
    qw_4 = msg.Transform.Rotation.W;
    
    x_4 = msg.Transform.Translation.X;
    y_4 = msg.Transform.Translation.Y;
    z_4 = msg.Transform.Translation.Z;
    
    %True/Zhipeng Transformations
    %x = (x+1.42)*200/1.8;
    %y = (y+0.12)*200/1.8;
    %z = (z-0.06)*100/1.5-100;
    
    %New Will Transformations (teenier)
    
    x_4 = (x_4+1.27)*200/1.5;
    y_4 = (y_4-.03)*200/1.5;
    z_4 = (z_4-0.16)*100/1.5-100;
end
        
%---------------------------- END ------------------------------------%