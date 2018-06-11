clear
load('Volt1.mat')
subplot(3,2,1)
plot(volt_1_save(1:11239,2),volt_1_save(1:11239,1))
xlabel('Time (sec)')
ylabel('Volts')
title('Battery 1 Discharge Trend')
clear

load('Volt2.mat')
subplot(3,2,2)
plot(volt_1_save(1:15106,2),volt_1_save(1:15106,1))
xlabel('Time (sec)')
ylabel('Volts')
title('Battery 2 Discharge Trend')
clear


load('Volt3.mat')
subplot(3,2,3)
plot(volt_1_save(1:12340,2),volt_1_save(1:12340,1))
xlabel('Time (sec)')
ylabel('Volts')
title('Battery 3 Discharge Trend')
clear

load('Volt4.mat')
subplot(3,2,4)
plot(volt_1_save(1:16201,2),volt_1_save(1:16201,1))
xlabel('Time (sec)')
ylabel('Volts')
title('Battery 4 Discharge Trend')
clear

load('Volt5.mat')
subplot(3,2,5)
plot(volt_1_save(1:12727,2),volt_1_save(1:12727,1))
xlabel('Time (sec)')
ylabel('Volts')
title('Battery 5 Discharge Trend')
clear

load('Volt6.mat')
subplot(3,2,6)
plot(volt_1_save(1:11382,2),volt_1_save(1:11382,1))
xlabel('Time (sec)')
ylabel('Volts')
title('Battery 6 Discharge Trend')
clear


