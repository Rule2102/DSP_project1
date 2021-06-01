clear,clc
close all

title_read = 'MSMUMAF270Hz_ok.mat'

load(title_read)

fpwm = 10e3;
N = 8;

[mag,phase] = bode(sys_estim);
mag = 20*log10(squeeze(mag));
phase = squeeze(phase);
f_test = sys_estim.Frequency;

figure(1)
subplot(2,1,1)
semilogx(f_test,mag,'-db','LineWidth',2')
subplot(2,1,2)
semilogx(f_test,phase,'-db','LineWidth',2')


alpha = 0.0636;
fs = N*fpwm;
Ts = 1/fs;

z=tf('z',Ts);
Wol = 0.25*(1+2*z^(-4)+z^(-8))*alpha/(z*(z-1));
f_test = (400:1:5e3);
w_test = 2*pi*f_test;
[mag,phase] = bode(Wol,w_test);
mag = 20*log10(squeeze(mag));
phase = squeeze(phase);

figure(1)
hold all
subplot(2,1,1)
hold all
semilogx(f_test,mag,'k','LineWidth',2')
subplot(2,1,2)
hold all
semilogx(f_test,phase-720,'k','LineWidth',2')

subplot(2,1,1)
legend('SFRA','$W_{ol,2}(z)$')
xlabel 'frequency [Hz]'
ylabel 'magnitude[dB]'
xlim([400 2.5e3])

subplot(2,1,2)
xlabel 'frequency [Hz]'
ylabel 'phase [deg]'
xlim([400 2.5e3])

