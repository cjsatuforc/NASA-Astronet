%------------------------ Define viconCallback -----------------------%
function viconCallback_Right_Arm(src,msg)
    global qx_right qy_right qz_right qw_right x_right y_right z_right;
    qx_right = msg.Transform.Rotation.X;
    qy_right = msg.Transform.Rotation.Y;
    qz_right = msg.Transform.Rotation.Z;
    qw_right = msg.Transform.Rotation.W;
    
    x_right = msg.Transform.Translation.X;
    y_right = msg.Transform.Translation.Y;
    z_right = msg.Transform.Translation.Z;
    
end
        
%---------------------------- END ------------------------------------%