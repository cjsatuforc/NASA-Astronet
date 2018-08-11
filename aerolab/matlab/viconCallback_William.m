%------------------------ Define viconCallback -----------------------%
function viconCallback_William(src,msg)
    global qx_head qy_head qz_head qw_head x_head y_head z_head;
    qx_head = msg.Transform.Rotation.X;
    qy_head = msg.Transform.Rotation.Y;
    qz_head = msg.Transform.Rotation.Z;
    qw_head = msg.Transform.Rotation.W;
    
    x_head = msg.Transform.Translation.X;
    y_head = msg.Transform.Translation.Y;
    z_head = msg.Transform.Translation.Z;
 
end
        
%---------------------------- END ------------------------------------%