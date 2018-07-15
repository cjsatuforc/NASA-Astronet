%------------------------ Define viconCallback -----------------------%
function viconCallback_hbirddg(src,msg)
    global qx_2 qy_2 qz_2 qw_2 x_2 y_2 z_2;
    qx_2 = msg.Rotation.X;
    qy_2 = msg.Rotation.Y;
    qz_2 = msg.Rotation.Z;
    qw_2 = msg.Rotation.W;
    
    x_2 = msg.Translation.X;
    y_2 = msg.Translation.Y;
    z_2 = msg.Translation.Z;
end
        
%---------------------------- END ------------------------------------%