%----------------------- Define statusCallback -----------------------%
function statusCallback_3(src,msg)
    global volt_3;
    volt_3 = msg.BatteryVoltage;

end
        
%---------------------------- END ------------------------------------%