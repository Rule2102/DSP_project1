clear,clc
%close all
z=tf('z');


%% note these parameters result in a nice solution. Do not change them.
SETTLE = 0.02
w = 0.2 : (1.57 - 0.2)/1000 : 1.57;

%good result is obtained using bw_min = 1.1, bw_target = 1.25, peak_target = 3;
bw_min = 1.1;
bw_target = 1.25;   
weight_speed = 10;
peak_target = 3;  %resonant peak in T, in dB

searchnew

num = [ a*(1+d)  -a*d];
den = [1  -1  a*(1+d)  -a*d];

%%
figure(1)
dstep(num,den)
hold on
step(T2) %Buso
step(T_mod_wx)
legend('optimized to same -3dB','Buso','a = 0.3, d =1')
grid

figure(2)
dbode(num,den,1)
hold on
bode(T2)
bode(T_mod_wx)
legend('optimized to same -3dB','Buso','a = 0.3, d =1')
grid

