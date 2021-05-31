%
clear all
clc

ur = 8;             % update rate
OVERSAMPLING = 1;   % logic variable to determine with/without oversampling

Id_ref = 0;                 % reference current in d axis 
Iq_ref = 2;                 % reference current in q axis

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

% scheduling
Ts_cmp = Ts;
Ts_exe = Ts; %[Ts Ts-1230*Tsim];
Tadc_exe = Tadc; %[Tadc mod(Ts-1230*Tsim,Tadc)];

% inverter parameters
E = 520;                   % available voltage
DEADTIME = 100;            % deadtime in multiples of Tpwm

% load parameters
R = 0.47;                   % phase resistance
L = 3.4e-3;                 % phase inductance
taus = L/R;                 % phase time constant
f_ref = 50;                % electrical frequency
we = 2*pi*f_ref;            % angular frequency
p = 3;                      % number of machine pole pairs
wm = we/p;                  % rotor mechanical speed
psi = 0.687/(sqrt(3)*p);                % permanenet magnet flux
emf = psi*we;               % machine electromotive force
ph = pi;                    % phase of the load when modeled with 3phase sine source (so that Ed=0)

% IREG parameters
% will be automatically adjusted if ur=8 (below)
alpha = 0.25; %0.38; %0.350;                % gain
K1 = alpha*L/Ts;            % constant for IREG
K2 = exp(-Ts/taus);         % parameter that desxcribes system dynamics
Udq_max = E/2;              % saturation level in dq (for IREG)
d = 0; %0.444; %0.4

% open loop dq voltages
Udol = 0;                   % d axis voltage
Uqol = 0;                   % q axis voltage

% parameters for measurements
ISENSE_SCALE = 10;          % scaling factor for current measurement
tau_fil = 5e-6;             % time const of low-pass filter
ofst_a = 0;           % offset for Ia (measurement error influence)
ofst_b = 0;           % offset for Ib (measurement error influence)

% initial conditions
Edq0_c = 1j*emf;                % complex electromotive force
Udq0_c = Edq0_c*exp(1j*we*Ts);  % complex dq voltage delayed for Ts   
Udq0 = [real(Udq0_c); imag(Udq0_c)];
Udq0 = [0; 0]; 
Uab0_c = Edq0_c*exp(1j*we*Ts*0);
Uab0 = [real(Uab0_c); imag(Uab0_c)];
Uabc0 = [1,0;-1/2,sqrt(3)/2;-1/2,-sqrt(3)/2]*Uab0;

% time steps of interest
dmacnt_ref = round(15*taus/Ts);      % regulation period in which reference step change occurs
tref = dmacnt_ref*Ts;               % reference step 
dmacnt_prnt = round(0.75*dmacnt_ref);                  % start printing in DSP (600 for ur=2)
dmacnt_end = round(dmacnt_ref*1.1);        % number of regulation periods for simulation to run
tend = dmacnt_end*Ts;               % simulation duration
MAX_data_count = dmacnt_end - dmacnt_prnt;   % array length for DSP's RAM export

if(ur==8)
    alpha = 0.0636; %0.12038; %0.15; %0.0636; %0.091; %;
    d = 0; %2.1948; %0.78975; % 0.5*Tpwm/Ts;
    %{
    dmacnt_prnt = 4000; %4*600; %
    MAX_data_count = 850; %4*443; %
    dmacnt_end = MAX_data_count + dmacnt_prnt;
    tend = dmacnt_end*Ts;
    %}
end

% frequency response analysis settings
fmax = 1/Tpwm/2;
f_test = 400:200:fmax;
z=tf('z',Ts);
W1 = alpha/(z^2-z+alpha);
tfra = 1.5; %0.33; % required experiment length based on f_test (see FRA block)
tend = tref + tfra;

% tuning PI IREG
wc = ur/2*3.01e3;
pm = 77.1*pi/180;
Kp = R*sqrt(1+(wc*L/R)^2);
Ki = wc*Kp/(tan(-pi/2+pm+2*atan(wc*Ts/4)+atan(wc*L/R)));


