%% DS-DU

% analytics
Wol = alpha*tf([1],[1 -1 0],Ts);
opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseWrapping = 'on';
opts.PhaseWrappingBranch = -360;
figure();
bode(Wol,f_test*2*pi,opts);
xlim([400 1/(2*Tpwm)]);
% [Gm,Pm,Wcg,Wcp] = margin(Wol);
% Wcp/(2*pi)
% Pm

% Simulink FRA
logdata = simout;
sys_estim = frestimate(logdata,f_test,'Hz');
hold all
bode(sys_estim,'b*');

% format figure
legend('Analytical','Simulated');
title(['OLFRA DS-DU \alpha = ' num2str(alpha)]);


%% MS-DU

% analytics
Wol = alpha*tf([1 2 1],[4 -4 0 0 0],Ts);
opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseWrapping = 'on';
opts.PhaseWrappingBranch = -360;
figure();
bode(Wol,f_test*2*pi,opts);
xlim([400 1/(2*Tpwm)]);
% [Gm,Pm,Wcg,Wcp] = margin(Wol);
% Wcp/(2*pi)
% Pm

% Simulink FRA
logdata = simout;
sys_estim = frestimate(logdata,f_test,'Hz');
hold all
bode(sys_estim,'b*');

% format figure
legend('Analytical','Simulated');
title(['OLFRA MS-DU \alpha = ' num2str(alpha)]);
%% MS-MU with MAF

% analytics
num_ol = [alpha 0 0 0 2*alpha 0 0 0 alpha];
den_ol = [4 -4 0 0 0 0 0 0 0 0 0];
Wol= tf(num_ol,den_ol,Ts);
opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseWrapping = 'on';
opts.PhaseWrappingBranch = -360;
figure();
bode(Wol,f_test*2*pi,opts);
xlim([400 1/(2*Tpwm)]);
% [Gm,Pm,Wcg,Wcp] = margin(Wol);
% Wcp/(2*pi)
% Pm

% Simulink FRA
logdata = simout;
sys_estim = frestimate(logdata,f_test,'Hz');
hold all
bode(sys_estim,'b*');

% format figure
legend('Analytical','Simulated');
title(['OLFRA MS-MU with MAF \alpha = ' num2str(alpha)]);