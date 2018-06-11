%------------------------ Define viconCallback -----------------------%
function viconCallback_hbirddg(src,msg)
    global qx_2 qy_2 qz_2 qw_2 x_2 y_2 z_2;
    qx_2 = msg.Pose.Orientation.X;
    qy_2 = msg.Pose.Orientation.Y;
    qz_2 = msg.Pose.Orientation.Z;
    qw_2 = msg.Pose.Orientation.W;
    
    x_2 = msg.Pose.Position.X;
    y_2 = msg.Pose.Position.Y;
    z_2 = msg.Pose.Position.Z;
end
        
%---------------------------- END ------------------------------------%