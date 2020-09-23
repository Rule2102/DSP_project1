%
clear all
clc

ur = 2;             % update rate
OVERSAMPLING = 1;   % logic variable to determine with/without oversampling

%}

Nos = 16;           % number of samples to be averaged on switching period
% if OVERSAMPLING then Nos is the oversampling rate

fcpu = 200e6;       % CPU clock
ftbclk = fcpu/2;    % EPWM time base clock frequency
Ttbclk = 1/ftbclk;  % EPWM time base clock period
fpwm = 10e3;        % desierd switching frequency

% PWM_TBPRD period of switching counter = floor(ftbclk/(2*fpwm) - 1) 
PWM_TBPRD = 4992; % !!!!! ADJUSTED FOR Nos=16 (4992 instead of 4998)
Tpwm = 2*PWM_TBPRD/ftbclk;      % resulting switching period
Ts = Tpwm/ur;                   % regulation period
Tadc = Tpwm/Nos;

% simulation step size
Tsim = gcd(gcd(Ttbclk*1e9,Tadc*1e9),Ts*1e9)/1e9; 

E = 650;                  % available voltage
DEADTIME = 100;

Udq_max = E/(2*sqrt(2));
UD_MAX = Udq_max;
UD_MIN = -Udq_max;
UQ_MAX = Udq_max;
UQ_MIN = -Udq_max;

Rs = 13.12;
Rr = 11.202;
Lgs = 0.0276;
Lgr = 0.0194;
Lm = 0.3482;
Ls = Lm + Lgs;
Lr = Lm + Lgr;
Lge = (Ls*Lr-Lm^2)/Lr;
J = 15e-3;
p = 1;
ID_NOM = 1; 

ISENSE_SCALE = 10;

Id_ref = ID_NOM;
Iq_ref = 1;
f_ref = 50;

alpha = 0.04;
K1 = alpha*Ls/Ts;
K2 = exp(-Ts*Rs/Ls);

Udol = UD_MAX/2;
Uqol = 0;



