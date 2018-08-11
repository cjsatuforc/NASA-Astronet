%------------------------ Define viconCallback -----------------------%
function viconCallback_1(src,msg)
    global qx_1 qy_1 qz_1 qw_1 x_1 y_1 z_1;
    qx_1 = msg.Transform.Rotation.X;
    qy_1 = msg.Transform.Rotation.Y;
    qz_1 = msg.Transform.Rotation.Z;
    qw_1 = msg.Transform.Rotation.W;
    
    x_1 = msg.Transform.Translation.X;
    y_1 = msg.Transform.Translation.Y;
    z_1 = msg.Transform.Translation.Z;
    
   %Larger Ones
    x_1 = (x_1+1.775)*200/2.55;
    y_1 = (y_1+0.345)*200/2.55;
    z_1 = (z_1-0.2)*100/2.3-100;
end
        
%---------------------------- END ------------------------------------%