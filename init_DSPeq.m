clear all
clc

Nos = 16;           % oversampling rate
ur = 2;             % update rate
% specify RC filter to be simulated
ADCINA0 = 1;
ADCINB2 = ~ADCINA0;

Tcpu = 1/200e6;     % CPU clock
Ttbclk=Tcpu*2;      % ePWM TB clock
Tpwm=1/10e3;        % switching period
Ts=Tpwm/ur;         % regulation period
if(Nos~=1)
    Tadc=Tpwm/Nos;  % ADC period
    % Switching PWM counter period
    TBPRD=(round(Tadc/(2*Ttbclk))-1)*Nos;       
else
    Tadc=Tpwm/ur;
    TBPRD=(round(Tadc/(2*Ttbclk))-1)*ur;
end
% simulation step size
Tsim = gcd(gcd(Ttbclk*1e9,Tadc*1e9),Ts*1e9)/1e9; 

E=3.3;              % available voltage

% RC connected to ADCINA0
if(ADCINA0)
    R=9.92e3;            % resistance
    C=130e-9;            % capacitance
    Rp=50e3;             % capacitor lekeage
else
    % RC connected to ADCINB2
    if(ADCINB2)
        R=9.92e3;        % resistance
        C=110e-9;        % capacitance
        Rp=1000e3;       % capacitor lekeage
    end
end

Kp=1.8;             % proportional gain
Ki=0.2/ur;          % integral gain

% step change of reference coltage
Vref1=0;
Vref2=1;
