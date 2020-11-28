% Case 1 (our controller VS. UR=2 no MAF

%% closed loop

% Analytical tf UR=2 no MAF
a = 0.25;
Ts_ur2 = Ts*4;
Wcl = a*tf([1],[1 -1 a],Ts_ur2);

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';

figure();
bode(Wcl,f_test*2*pi,opts);
xlim([400 1/(2*Tpwm)]);

% Analytical tf UR=8 with MAF
a_nas = 0.0636;
d_nas = 0;
Ts_nas = Ts;

num_nas = [4*a_nas*(1+d_nas) -4*a_nas*d_nas 0 0 0 0 0 0 0 0];
den_nas = [4 -4 a_nas*(1+d_nas) -a_nas*d_nas 0 0 2*a_nas*(1+d_nas) -2*a_nas*d_nas 0 0 a_nas*(1+d_nas) -a_nas*d_nas];
Wcl_nas = tf(num_nas,den_nas,Ts_nas);

hold all;
bode(Wcl_nas,f_test*2*pi,opts);
xlim([400 1/(2*Tpwm)]);

% FRA UR=8 with MAF
logdata = simout;
sys_estim = frestimate(logdata,f_test,'Hz');
%sys_estim = frestimate(logdata,f_test*2*pi,'rad/s');

hold all
bode(sys_estim,'b*');
legend('Analytical UR=2 no MAF','Analytical UR=8 with MAF','Simulated UR=8 with MAF');

%% open loop

% Analytical tf UR=2 no MAF
a = 0.25;
Ts_ur2 = Ts*4;
Wol = a*tf([1],[1 -1 0],Ts_ur2);

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseWrapping = 'on';
opts.PhaseWrappingBranch = -360;

figure();
bode(Wol,f_test*2*pi,opts);
xlim([400 1/(2*Tpwm)]);

% Analytical tf UR=8 with MAF
a_nas = 0.0636;
d_nas = 0;
Ts_nas = Ts;

numpp_nas = a_nas*[(1+d_nas) -d_nas 0 0 2*(1+d_nas) -2*d_nas 0 0 (1+d_nas) -d_nas];
denpp_nas = [4 -4 0 0 0 0 0 0 0 0 0 0];
Wol_nas = tf(numpp_nas,denpp_nas,Ts_nas);

hold all;
bode(Wol_nas,f_test*2*pi,opts);
xlim([400 1/(2*Tpwm)]);

% FRA UR=8 with MAF
logdata = simout;
sys_estim = frestimate(logdata,f_test,'Hz');
%sys_estim = frestimate(logdata,f_test*2*pi,'rad/s');

hold all
bode(sys_estim,'b*');
legend('Analytical UR=2 no MAF','Analytical UR=8 with MAF','Simulated UR=8 with MAF');
