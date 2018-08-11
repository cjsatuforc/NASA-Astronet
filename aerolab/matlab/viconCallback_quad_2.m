%------------------------ Define viconCallback -----------------------%
function viconCallback_quad(src,msg)
    global qx_2 qy_2 qz_2 qw_2 x_2 y_2 z_2;
    qx_2 = msg.Transform.Rotation.X;
    qy_2 = msg.Transform.Rotation.Y;
    qz_2 = msg.Transform.Rotation.Z;
    qw_2 = msg.Transform.Rotation.W;
    
    x_2 = msg.Transform.Translation.X;
    y_2 = msg.Transform.Translation.Y;
    z_2 = msg.Transform.Translation.Z;
    
end

%---------------------------- END ------------------------------------%