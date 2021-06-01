clear,clc
close all

title_read = 'MSDUstepHIL.mat'

load(title_read)

t = t*1e6;      % time in us
t = t + 100;    % start from 0

figure(1)
grid
hold all
stairs(t,Iqan,'k','LineWidth',1)
stairs(t,Iqref,'b--','LineWidth',1)
plot(t,Iq,'-bd','LineWidth',2)
plot(t,Id,'r','LineWidth',2)
legend('$W_{cl}(z)$','$i_q^*$','$i_q$','$i_d$')
xlabel 'time [us]'
ylabel 'current [A]'
xlim([0 1100])

