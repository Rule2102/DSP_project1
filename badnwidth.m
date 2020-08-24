Ts = 50e-6;     % regulation period
z=tf('z',Ts);
alfa = 0.2;     % controller gain

% closed loop transfer function 
Wcl_OS0 = alfa/(z^2-z+alfa);                                % NO oversampling
Wcl_OS1 = 4*alfa*z^2/(4*z^4-4*z^3+alfa*z^2+2*alfa*z+alfa);  % WITH oversampling

% Bode plots
figure();
bode(Wcl_OS0);
hold all; 
bode(Wcl_OS1);
legend('OVERSAMPLING = 0', 'OVERSAMPLING = 1','Location','southeast');

% bandwidth
BW_OS0 = bandwidth(Wcl_OS0)/(2*pi)*Ts;
BW_OS1 = bandwidth(Wcl_OS1)/(2*pi)*Ts;

% overshoot
s_OS0=stepinfo(Wcl_OS0);
overshoot_OS0 = s_OS0.Overshoot;
s_OS1=stepinfo(Wcl_OS1);
overshoot_OS1 = s_OS1.Overshoot;

%step response
figure();
step(Wcl_OS0);
hold all;
step(Wcl_OS1);
legend('OVERSAMPLING = 0', 'OVERSAMPLING = 1','Location','southeast');