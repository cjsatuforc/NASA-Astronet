%------------------------ Define xyzCallback -----------------------%
function xyzCallback_1(src,msg)
    global x_1 y_1 z_1;
    global long_1 lat_1 alt_1;
    global pos_cov_1 type_1;
    
    long_1 = msg.Longitude;
    lat_1 = msg.Latitude;
    alt_1 = msg.Altitude;
    
    for i=1:9
       pos_cov_1(i) = msg.PositionCovariance(i);
    end
    
    type_1 = msg.PositionCovarianceType;
    
    x_1 =  lat_1-(42.293236100000001);
    y_1 = -(long_1)-(83.711706100000001);
    z_1 = alt_1-241.7-2;   
    
    x_1 = ((x_1)*111078.9538); % convert to meters only
    y_1 = ((y_1)*82469.093);
    z_1 = (z_1);
    
end
        
%---------------------------- END ------------------------------------%
