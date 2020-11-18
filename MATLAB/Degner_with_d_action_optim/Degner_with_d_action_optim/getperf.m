% Get Performance -- Single Sample

% Degner, Wobj * W_basic_controller = Alpha/z/(z-1)       
% num den closed loop
% numpp denpp open loop
          
% Degner,  Single Sample, no-differential action, from his paper          
          num = a; 
          den = [1  -1  a]; 
          numpp = a;
          denpp = [ 1 -1 0];

% Degner Controller with Differential multiplier (1+d)-d(1/z)
          %num = [ a*(1+d)  -a*d];; 
          %den = [1  -1  a*(1+d)  -a*d]; 
          %numpp = [ a*(1+d)  -a*d];
          %denpp = [ 1 -1 0 0];

          
%-----------------------------------------------------------
          iout = dstep(num,den,50); ierr = abs(iout - 1); 
          perf = max(find(ierr>SETTLE));