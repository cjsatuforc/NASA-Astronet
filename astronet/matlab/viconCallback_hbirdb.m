%------------------------ Define viconCallback -----------------------%
function viconCallback_hbirdb(src,msg)
    global qx_1 qy_1 qz_1 qw_1 x_1 y_1 z_1;
    qx_1 = msg.Rotation.X;
    qy_1 = msg.Rotation.Y;
    qz_1 = msg.Rotation.Z;
    qw_1 = msg.Rotation.W;
    
    x_1 = msg.Translation.X;
    y_1 = msg.Translation.Y;
    z_1 = msg.Translation.Z;

end
        
%---------------------------- END ------------------------------------%