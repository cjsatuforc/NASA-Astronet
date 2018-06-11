%----------------------- Define statusCallback -----------------------%
function statusCallback_4(src,msg)
    global volt_4;
    volt_4 = msg.BatteryVoltage;

end
        
%---------------------------- END ------------------------------------%