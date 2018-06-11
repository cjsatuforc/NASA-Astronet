%------------------------ Define viconCallback -----------------------%
function viconCallback_quad(src,msg)
    global qx_1 qy_1 qz_1 qw_1 x_1 y_1 z_1;
    qx_1 = msg.Transform.Rotation.X;
    qy_1 = msg.Transform.Rotation.Y;
    qz_1 = msg.Transform.Rotation.Z;
    qw_1 = msg.Transform.Rotation.W;
    
    x_1 = msg.Transform.Translation.X;
    y_1 = msg.Transform.Translation.Y;
    z_1 = msg.Transform.Translation.Z;
    
 
 
end
        
%---------------------------- END ------------------------------------%