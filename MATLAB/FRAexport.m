figure();
bode(W1,f_test*2*pi,'r');
hold all
logdata = out.simout;
sys_estim = frestimate(logdata,f_test*2*pi,'rad/s');
bode(sys_estim,'b*');
legend('original','estimated');
