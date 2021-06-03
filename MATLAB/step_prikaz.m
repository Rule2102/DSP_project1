%% DS-DU

% analytics
Ts = 1000*Ts; % to plot in [ms]
Wcl = alpha*tf([1],[1 -1 alpha],Ts);
opt = stepDataOptions('InputOffset',4,'StepAmplitude',-2);
kl = 20 - 1; % length of time vector (*Ts)
ks = 2174; % reference step change occurs at ks*Ts

figure();
title(['DS-DU \alpha = ' num2str(alpha)]);

%% MS-DU

% analytics
Ts = 1000*Ts; % to plot in [ms]
Wcl = tf([4*alpha 0 0],[4 -4 alpha 2*alpha alpha],Ts);
Wfb = tf([1 2 1],[4 0 0],Ts);
%Wcl = Wcl*Wfb;
opt = stepDataOptions('InputOffset',4,'StepAmplitude',-2);
kl = 20 - 1; % length of time vector (*Ts)
ks = 2174; % reference step change occurs at ks*Ts

figure();
title(['MS-DU \alpha = ' num2str(alpha)]);

%% MS-MU with MAF

% analytics
Ts = 1000*Ts; % to plot in [ms]
num = [4*alpha 0 0 0 0 0 0 0 0];
den = [4 -4 alpha 0 0 0 2*alpha 0 0 0 alpha];
Wcl= tf(num,den,Ts);
opt = stepDataOptions('InputOffset',4,'StepAmplitude',-2);
kl = 60 - 1; % length of time vector (*Ts)
ks = 8695; % reference step change occurs at ks*Ts

figure();
title(['MS-MU with MAF \alpha = ' num2str(alpha)]);

%% plot (used for all control loop architectures)
tstep = (-1:1:kl)*Ts;
kn = length(tstep);
yq = step(Wcl,tstep,opt);
yq = [4; yq];
yd = zeros(1,kn);

% format figure
xl = xlabel('Time [ms]');
set(xl, 'Units', 'Normalized', 'Position', [0.5, 0.07, 0]);
xlim([tstep(1)-Ts tstep(end)]);
grid on
yyaxis left
stairs(tstep,yq);
hold all
yl = ylabel('Iq [A]');
set(yl, 'Units', 'Normalized', 'Position', [0.07, 0.3, 0]);
ylim([1.5 4.5]);

% load data

% load from scope
% Id = Idq(:,2);
% Iq = Idq(:,3);
% Id_ref = Idq2(:,2);
% Iq_ref = Idq2(:,3);

% load from file
load('MSMUstep.mat','Id','Iq','Id_ref','Iq_ref');

% Simulink results
Ids = Id(ks:(ks+(kl+1)));
Iqs = Iq(ks:(ks+(kl+1)));
plot(tstep,Iqs);

% references
Idrefs = Id_ref(ks:ks+kl+1);
Iqrefs = Iq_ref(ks:ks+kl+1);
stairs(tstep,Iqrefs,'LineWidth',1.3);

yyaxis right
stairs(tstep,yd);
plot(tstep,Ids);
stairs(tstep,Idrefs,'LineWidth',1.3);
ylim([-1.5 1.5]);
yl = ylabel('Id [A]');
set(yl, 'Units', 'Normalized', 'Position', [0.93, 0.3, 0]);

legend('I_q^{an}','I_{q}^{sym}','I_q^{ref}','I_d^{an}','I_d^{sym}','I_d^{ref}');
Ts = Ts/1000;















