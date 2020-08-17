%
clear all
clc

ur = 1;             % update rate
OVERSAMPLING = 0;   % logic variable to determine with/without oversampling

% specify RC filter to be simulated
ADCINA0 = 1;
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

E=3.3;                  % available voltage

% RC connected to ADCINA0
if(ADCINA0)
    R=9.92e3;            % resistance 
    C=130e-9;            % capacitance 130e-9
    Rp=51e3;             % capacitor lekeage
else
    % RC connected to ADCINB2
    R=9.92e3;        % resistance
    C=110e-9;        % capacitance
    Rp=1000e3;       % capacitor lekeage
end

beta = exp(-Ts/(R*C));      % parametar that describes the system 
alfa = 0.2;                 % gain for IMC based regulator

% !!! use switch in Simulink regulator block to set PI ot IMC regulator
% parameters for PI regulator 
Kp= 1.8;              % proportional gain
Ki= 0.2/ur;           % integral gain

% step change of reference coltage
Vref1=0;
Vref2=0.5;

% defines simulation length (synchronized with DSP memory storage)
MAX_data_count = 180;



