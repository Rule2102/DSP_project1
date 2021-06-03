clear all
ur = 2;             % update rate
fcpu = 200e6;       % CPU clock
ftbclk = fcpu/2;    % EPWM time base clock frequency
Ttbclk = 1/ftbclk;  % EPWM time base clock period
fpwm = 10e3;        % desierd switching frequency

% PWM_TBPRD period of switching counter = floor(ftbclk/(2*fpwm) - 1) 
PWM_TBPRD = 4992; % !!!!! ADJUSTED FOR Nos=16 (4992 instead of 4998)
Tpwm = 2*PWM_TBPRD/ftbclk;      % resulting switching period
Ts = Tpwm/ur;                   % regulation period


%% DS-DU
alpha = 0.23;
Wol = alpha*tf([1],[1 -1 0],Ts);
[Gm,Pm,Wcg,Wcp] = margin(Wol);
Pm
Wcp/2/pi

Wcl = alpha*tf([1],[1 -1 alpha],Ts);
bandwidth(Wcl)/(2*pi)

%% MS-DU
alpha = 0.14;
Wol = alpha*tf([1 2 1],[4 -4 0 0 0],Ts);
[Gm,Pm,Wcg,Wcp] = margin(Wol);
Pm
Wcp/2/pi

Wcl = tf([4*alpha 0 0],[4 -4 alpha 2*alpha alpha],Ts);
bandwidth(Wcl)/(2*pi)

%% MS-MU-MAF
alpha = 0.0636;
num_ol = [alpha 0 0 0 2*alpha 0 0 0 alpha];
den_ol = [4 -4 0 0 0 0 0 0 0 0 0];
Wol= tf(num_ol,den_ol,Ts);
[Gm,Pm,Wcg,Wcp] = margin(Wol);
Pm
Wcp/2/pi

num = [4*alpha 0 0 0 0 0 0 0 0];
den = [4 -4 alpha 0 0 0 2*alpha 0 0 0 alpha];
Wcl= tf(num,den,Ts);
bandwidth(Wcl)/(2*pi)

%% MS-MU
alpha = 0.04;
Wol = alpha*tf([1],[1 -1 0],Ts);
[Gm,Pm,Wcg,Wcp] = margin(Wol);
Pm
Wcp/2/pi