%
clear all
clc

ur = 2;             % update rate
OVERSAMPLING = 0;   % logic variable to determine with/without oversampling

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

E = 50; %4; %520;                 % available voltage
DEADTIME = 100;

Udq_max = E/(2);
UD_MAX = Udq_max;
UD_MIN = -Udq_max;
UQ_MAX = Udq_max;
UQ_MIN = -Udq_max;

R = 0.47; %0.0307; % 0.47;
L = 3.4e-3; %120e-6; %10e-3;

ISENSE_SCALE = 10;

Id_ref = 4; %0; %5;
Iq_ref = 0; %10;
f_ref = 100; %150;
we = 2*pi*f_ref;

alpha = 0.2; %0.04;
K1 = alpha*L/Ts;
K2 = exp(-Ts*R/L);

Udol = 0.1; %UD_MAX/2;
Uqol = 0;

tau_fil = 5e-6;





