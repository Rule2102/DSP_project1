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
load('MSMUolfra.mat');

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
axes(ha(1)); % ax1 = subplot(2,1,1); % gain plot
    plot(wout/2/pi, 20*log10(mag(:)));
    set(gca, 'XScale', 'log');
    grid on
    set(gca,'xticklabel',{[]}); %[500 1000 1500 2500 3500 5000]); %xticks([]);
    yl = ylabel('Magnitude [dB]');
    set(yl, 'Units', 'Normalized', 'Position', [0.05, 0.5, 0]);
    ylim([-22 10]);
axes(ha(2)); %ax2 = subplot(2,1,2); % Phase plot 
    plot(wout/2/pi, phase(:));
    set(gca, 'XScale', 'log');
    grid on
    xl = xlabel('Frequency [Hz]');
    set(xl, 'Units', 'Normalized', 'Position', [0.5, 0.15, 0]);
    xticks([500 1000 1500 2500 3500 5000]);
    yl = ylabel('Phase [^\circ]');
    set(yl, 'Units', 'Normalized', 'Position', [0.05, 0.5, 0]);
    ylim([-225 -90]);

[mag,phase,wout] = bode(sys_estim);
axes(ha(1)); %ax1 = subplot(2,1,1); % gain plot
    hold all
    plot(wout/2/pi, 20*log10(mag(:)),'b*');
axes(ha(2)); %ax2 = subplot(2,1,2); % Phase plot 
    hold all
    plot(wout/2/pi, phase(:),'b*');
    legend('Analytical','Simulated');
    
axes(ha(1)); %ax1 = subplot(2,1,1); % gain plot
title(tit);

%% format Bode plots (for all controller architectures)
pom = gca;

axes=findobj('type','axes');
h_magnitude=get(axes(2),'YLabel');
h_phase=get(axes(1),'YLabel');
%h_x= get(axes(1),'XLabel');
set(h_x,'String','Frequency [Hz]');
set(h_x,'Units', 'Normalized', 'Position', [0.5, 0.77, 0]);
set(h_magnitude,'Units','Normalized','Position',[0.07, 0.5, 0]);
set(h_magnitude,'String','Magnitude [dB]');
set(h_phase,'Units','Normalized','Position',[0.07, 0.5, 0]);
set(h_phase,'String','Phase [^\circ]');