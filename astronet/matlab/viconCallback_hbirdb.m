%------------------------ Define viconCallback -----------------------%
function viconCallback_hbirdb(src,msg)
    global qx_1 qy_1 qz_1 qw_1 x_1 y_1 z_1;
    qx_1 = msg.Pose.Orientation.X;
    qy_1 = msg.Pose.Orientation.Y;
    qz_1 = msg.Pose.Orientation.Z;
    qw_1 = msg.Pose.Orientation.W;
    
    x_1 = msg.Pose.Position.X;
    y_1 = msg.Pose.Position.Y;
    z_1 = msg.Pose.Position.Z;

end
        
%---------------------------- END ------------------------------------%