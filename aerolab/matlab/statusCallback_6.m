%----------------------- Define statusCallback -----------------------%
function statusCallback_6(src,msg)
    global volt_6;
    volt_6 = msg.BatteryVoltage;

end
        
%---------------------------- END ------------------------------------%