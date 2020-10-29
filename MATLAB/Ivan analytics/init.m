clear,clc
% set [Hz] as default freqeuncy unit
set(cstprefs.tbxprefs,'FrequencyUnits', 'Hz')

s=tf('s')

fpwm = 10e3;
Tpwm = 1/ fpwm;
N = 8;
averaging = 0;
derivative = 0;

fs = N*fpwm;
Ts = 1/fs;
z = tf('z',Ts);

we = 2*pi*50;
R = 1;
L = 5e-3;

%% controller parameters - both s-domain based design and direct discrete design

% bw = 0.17 * 2 * fpwm
alpha_z = 0.2        %% for N = 2 -> 0.2 / Ts , for N = 8 -> 0.087 / Ts
alpha_s = alpha_z  / Ts;

kp = L;
ki = R;
kc = we*L;

Tsim = 200e-3;

if averaging == 1
    Te = 1.5 * Ts + 0.5*Tpwm;
else
    Te = 1.5 * Ts;
end

if derivative == 1
    Gzero_s = (1 + s * 0.5*Tpwm);
    Gzero_z = (1 + (1-1/z) * 0.5*Tpwm / Ts);
else
    Gzero_s = 1;
    Gzero_z = 1;
end

%% s-domain analysis - napravi matricno da bi mogao da gledas i coupling. Ovde zeza s-domain modelling kada hocu jako velike propusne opsege (koji su blizu Tpwm/2), zbog ubacivanja ove nule. Ovo mozda moram da uradim u z-domenu.
Tol_s = (alpha_s / s) * exp(-s* 1.5 * Ts) * Gzero_s;
Tcl_s = feedback(Tol_s,exp(-s*0.5*Tpwm));

% figure(1)
% hold all
% margin(Tol_s)
% 
% figure(2)
% hold all
% bode(Tcl_s)
% xlim([10 fpwm/2])
% 
% figure(3)
% hold all
% step(Tcl_s)
%% z-domain analysis - TF from Vuksa, TF from comment on Hoffman

Tol_z = Gzero_z * alpha_z / (z * (z - 1));
MAF_approximate = (1 + 2/z^(N/2) + 1/(z^N)) / 4;
Tcl_z = feedback(Tol_z,MAF_approximate);

% figure(1)
% margin(Tol_z)
% 
% figure(2)
% bode(Tcl_z)
% 
% figure(3)
% step(Tcl_z)

%% exact plant model in z-domain
close all
clc
%%

tau = L / R; 
gama = exp(-Ts/tau - 1i*we*Ts);
K = (1 - exp(-Ts/tau))*exp(-2*1i*we*Ts) / R;
L_withoutGc = K / (z * (z - gama));

Gc_IMC_z = (alpha_z / (z * (z-1))) / L_withoutGc;  %% inverting dynamics, as in Vuksa's papers (ZOH modelling of DPWM)
Gc_Sbased_z = alpha_z * ( R*z/(z-1) + L/Ts +1i*we*L*z/(z-1) ) * exp(1.5*1i*we*Ts);  %% s-domain based IMC -> DPWM modelled as exp(-sTs/2) to enable invert of phase lag

L_IMC_z = Gc_IMC_z * L_withoutGc;
L_IMC_z = minreal(L_IMC_z)

L_Sbased_z = Gc_Sbased_z * L_withoutGc;
L_Sbased_z = minreal(L_Sbased_z)

T_IMC_z = feedback(L_IMC_z,1)       %% see from result whether imaginary part makes sense (i.e. if e-20, then it is something numerical...). I will make wxMaxima script to see as well!!!
T_Sbased_z = feedback(L_Sbased_z,1)

figure(1)
bode(T_IMC_z)
hold on
bode(T_Sbased_z)
figure(2)
step(T_IMC_z)
hold on
step(T_Sbased_z)


figure(3)
pzmap(T_IMC_z)
hold on
pzmap(T_Sbased_z)

%% controller with derivative gain. I want to fix derivative gain by my logic - compensate for filter delay, and then I want to search for a to find desired bandwidth (>35%fpwm with no overshoot)
% For N = 8
close all
clc

Gzero_z = (1 + (1-1/z) * 0.5*Tpwm / Ts);

f_test = 100:5:fs;
w_test = 2*pi*f_test;
x = [100 fpwm];
y = [-3 -3];

figure(1)
plot(x,y,'k--')
hold all
x = [fpwm*0.35 fpwm*0.35];
y = [-5 0];
figure(1)
plot(x,y,'k--')
grid
xlim([0 fpwm])

% for alpha_test = 0.35:0.005:0.4
%     alpha_z = alpha_test / (N / 2)
%     L_z = Gzero_z * alpha_z / (z * (z - 1));
%     T_z = feedback(L_z,MAF_approximate);
%     [mag,phase] = bode(T_z,w_test);
%     mag = 20*log10(squeeze(mag));
%     phase = squeeze(phase);
%     figure(1)
%     semilogx(f_test,mag)
%     
%     S_z = 1 - T_z * MAF_approximate; %% or maybe makes more sense 1 - T_z*MAF(z) but ok for now - see LATER
%     [mag,phase] = bode(S_z,w_test);
%     mag = 20*log10(squeeze(mag));
%     phase = squeeze(phase);
%     figure(2)
%     semilogx(f_test,mag)
%     hold all
% end
% FROM THIS, I get that for alpha_z 0.385 / 4 and my derivative gain, I get
% -3dB at 3790 Hz, which is 37.9 % pwm.
% For these parameters peak of S is equal
% to 4.964 dB at 20 kHz, and 2.532 at 4305 Hz. I am not sure what to
% consider phase margin, as above fpwm, we are hardly linear in any case.
% Therefore it is maybe better to use the one bellow fpwm for the
% multisampling case. NEXT STEP IS TO OPTIMIZE BOTH a AND d BASED ON THE
% STEP RESPONSE AND -3DB LIMIT
% VERY INTERESTING - IN VUKSA PAPER HIS GAIN ALPHA IS 0.38 -> MINE IS
% 0.385/4, AND HE USES DOUBLE UPDATE AND I N = 8. HIS d IS EQUAL TO 0.444
% AND MINE IS EQUAL TO 4, WHICH IS HIGHER THAN 0.444 * 4

%% plot step responses
% close all
% clc

% Gzero_z = (1 + (1-1/z) * 0.444 * 4);
alpha_z = 0.385 / 4;
L_z = Gzero_z * alpha_z / (z * (z - 1));

T_z = feedback(L_z,MAF_approximate);


figure(1)
step(T_z)
hold all

    