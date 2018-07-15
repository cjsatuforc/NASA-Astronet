%----------------------- Define statusCallback -----------------------%
function statusCallback_5(src,msg)
    global volt_5;
    volt_5 = msg.BatteryVoltage;

end
        
%---------------------------- END ------------------------------------%