%------------------------ Define viconCallback -----------------------%
function viconCallback_Left_Arm(src,msg)
    global qx_left qy_left qz_left qw_left x_left y_left z_left;
    qx_left = msg.Transform.Rotation.X;
    qy_left = msg.Transform.Rotation.Y;
    qz_left = msg.Transform.Rotation.Z;
    qw_left = msg.Transform.Rotation.W;
    
    x_left = msg.Transform.Translation.X;
    y_left = msg.Transform.Translation.Y;
    z_left = msg.Transform.Translation.Z;
    
end
        
%---------------------------- END ------------------------------------%