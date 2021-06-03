%% DS-DU

% analytics
Wol = alpha*tf([1],[1 -1 0],Ts);

% Simulink FRA
% logdata = simout;
% sys_estim = frestimate(logdata,f_test,'Hz');
load('DSDUolfra.mat');

figure()
tit = ['OLFRA DS-DU \alpha = ' num2str(alpha)];

%% MS-DU

% analytics
Wol = alpha*tf([1 2 1],[4 -4 0 0 0],Ts);

% Simulink FRA
% logdata = simout;
% sys_estim = frestimate(logdata,f_test,'Hz');
load('MSDUolfra.mat');

figure()
tit = ['OLFRA MS-DU \alpha = ' num2str(alpha)];
%% MS-MU with MAF

% analytics
num_ol = [alpha 0 0 0 2*alpha 0 0 0 alpha];
den_ol = [4 -4 0 0 0 0 0 0 0 0 0];
Wol= tf(num_ol,den_ol,Ts);

% Simulink FRA
% logdata = simout;
% sys_estim = frestimate(logdata,f_test,'Hz');
load('MSMUMAFolfra.mat');

figure()
tit = ['OLFRA MS-MU with MAF \alpha = ' num2str(alpha)];

%%

% analytical fc & pm
% [Gm,Pm,Wcg,Wcp] = margin(Wol);
% Wcp/(2*pi)
% Pm

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseWrapping = 'on';
opts.PhaseWrappingBranch = -360;

% use built-in Bode plots
% bode(Wol,f_test*2*pi,opts);
% xlim([400 1/(2*Tpwm)]);
%hold all
%bode(sys_estim,'b*');

[ha, pos] = tight_subplot(2,1,0.05,0.07,0.07);

%create your own Bode plots
[mag,phase,wout] = bode(Wol,f_test*2*pi,opts);
maga = squeeze(mag(1,1,:))';
phasea = squeeze(phase(1,1,:))'; %-720;
wouta = wout';

% without erronous point
% Nerr = 16;
% maga = [maga(1:Nerr-1), maga(Nerr+1:end)];
% phasea = [phasea(1:Nerr-1), phasea(Nerr+1:end)];
% wouta = [wouta(1:Nerr-1), wouta(Nerr+1:end)];

axes(ha(1)); % ax1 = subplot(2,1,1); % gain plot
    plot(wouta/2/pi, 20*log10(maga(:)));
    set(gca, 'XScale', 'log');
    grid on
    set(gca,'xticklabel',{[]}); %[500 1000 1500 2500 3500 5000]); %xticks([]);
    yl = ylabel('Magnitude [dB]');
    set(yl, 'Units', 'Normalized', 'Position', [0.05, 0.5, 0]);
%     ylim([-22 10]);
axes(ha(2)); %ax2 = subplot(2,1,2); % Phase plot 
    plot(wouta/2/pi, phasea(:));
    set(gca, 'XScale', 'log');
    grid on
    xl = xlabel('Frequency [Hz]');
    set(xl, 'Units', 'Normalized', 'Position', [0.5, 0.15, 0]);
    xticks([500 1000 1500 2500 3500 5000]);
    yl = ylabel('Phase [^\circ]');
    set(yl, 'Units', 'Normalized', 'Position', [0.05, 0.5, 0]);
%     ylim([-225 -90]);


[mag,phase,wout] = bode(sys_estim);
mag = squeeze(mag(1,1,:))';
phase = squeeze(phase(1,1,:))';
wout = wout';

% without erronous point
% Nerr = 16;
% mag = [mag(1:Nerr-1), mag(Nerr+1:end)];
% phase = [phase(1:Nerr-1), phase(Nerr+1:end)];
% wout = [wout(1:Nerr-1), wout(Nerr+1:end)];

axes(ha(1)); %ax1 = subplot(2,1,1); % gain plot
    hold all
    plot(wout/2/pi, 20*log10(mag(:)),'b*');
axes(ha(2)); %ax2 = subplot(2,1,2); % Phase plot 
    hold all
    plot(wout/2/pi, phase(:),'b*');
    legend('Analytical','Simulated');
    
axes(ha(1)); %ax1 = subplot(2,1,1); % gain plot
%title(tit);

fout = wout/2/pi;
maga = 20*log10(maga);
mag = 20*log10(mag);
save('DSDUolfra1.mat','maga','phasea','mag','phase','fout');
