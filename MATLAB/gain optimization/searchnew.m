close all; format short g; 

% SETTLE = 0.02;  %% this value will define our overshoot
IgnoreD = 0; 
  
R=0.47;  L = 0.0033;  T = 0.0000625; alfa = exp(-T/(L/R));
a = 0.25; d = 0.3; best_a = a; best_d = d; a0 = best_a; d0 = best_d;
best_perf = 1000; perf = best_perf; 

MIN = 0.2; MAX = 3; STEP = 0.05;
% Primo passo, ricerca grezza

for a = MIN*a0 : STEP*a0 : MAX*a0,
    [1 MIN a/a0 MAX perf best_perf best_a best_d],
   for d = MIN*d0 : STEP*d0 : MAX*d0,
        getperf; 
          if perf < best_perf,
              best_perf = perf; best_a = a; best_d = d; 
          end
   end
end

if IgnoreD == 1, best_d = d0; end; 
if ( (best_a <= MIN*a0) || (best_d <= MIN*d0) ), 
    disp(' ERROR WE ARE ON THE LOWER EDGE '); pause; 
end; 
if ( (best_a >= MAX*a0) || (best_d >= MAX*d0) ), 
    disp(' ERROR WE ARE ON THE UPPER EDGE '); pause; 
end;     

% **********************************************************************
% Secondo Giro 

a0 = best_a; d0 = best_d;
MIN = 0.7; MAX = 1.4; STEP = 0.01; 

for a = MIN*a0 : STEP*a0 : MAX*a0,
    [2 MIN a/a0 MAX perf best_perf best_a best_d],
   for d = MIN*d0 : STEP*d0 : MAX*d0,
        getperf; 
          if perf < best_perf,
              best_perf = perf; best_a = a; best_d = d; 
          end
   end
end

if IgnoreD == 1, best_d = d0; end;
if ( (best_a <= MIN*a0) || (best_d <= MIN*d0) ), 
    disp(' 2 ERROR WE ARE ON THE LOWER EDGE '); pause; 
end; 
if ( (best_a >= MAX*a0) || (best_d >= MAX*d0) ), 
    disp(' 2 ERROR WE ARE ON THE UPPER EDGE '); pause; 
end;     

%---------------------------------------------------------------------
fs_ivan=20e3;
a = best_a; d = best_d;
getperf;   % will get polinomials 
ioutout = dstep(num,den,50); 
Preb = max(ioutout); 
for jj=1:1000, W(jj)=jj*50; end; 
[Mag, Pha, W] = dbode(num,den,1/fs_ivan,W); 
fbw = W(min(find(Mag<0.707)))/2/pi;
f45 = W(min(find(Pha< -45)))/2/pi;
VMall; figure; 
stairs(ioutout); grid; 
disp('[ a d f45 fbw VM best_perf  Over]');

[ a d f45 fbw VM best_perf Preb]


% Degner,  Single Sample, no-differential action, from his paper
%                 a       d     f45       fbw     VM      best_perf  Over
% 2% settling    0.3      X    748.03    2069   0.6547        6     1.0119
% 1% settling    0.2875   X    724.15    1909   0.66769       7     1.0053
% 0.5%           0.27775  X    700.28    1790   0.67787       8     1.002

% Degner,  Single Sample, with differential action
%                 a       d     f45       fbw     VM      best_perf  Over
% 2% settling    0.375    0.27  946      3573   0.58          4     1.0172
% 1% settling    0.35     0.195 875      2904   0.61          5     1.0084
% 0.5%           0.35     0.21  883      2928   0.61          5     1.0047    
      

