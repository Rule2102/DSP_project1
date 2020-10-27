figure();
W1 = alpha/(z*(z-1));
f_test2 = 400:50:1/Tpwm;
bode(W1,f_test2*2*pi,'r');
hold all
logdata = out.simout;
sys_estim = frestimate(logdata,f_test*2*pi,'rad/s');
bode(sys_estim,'b*');
legend('original','estimated');
