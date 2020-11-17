% Get Performance -- Single Sample

% Degner, Wobj * W_basic_controller = Alpha/z/(z-1)
% num den closed loop
% numpp denpp open loop

% Degner,  Single Sample, no-differential action, from his paper
% num = a;
% den = [1  -1  a];
% numpp = a;
% denpp = [ 1 -1 0];

% Degner Controller with Differential multiplier (1+d)-d(1/z)
num = [ a*(1+d)  -a*d];;
den = [1  -1  a*(1+d)  -a*d];
numpp = [ a*(1+d)  -a*d];
denpp = [ 1 -1 0 0];
          
          
%-----------------------------------------------------------
iout = dstep(num,den,50); ierr = abs(iout - 1);

[MAG,PHASE] = dbode(num,den,1,w);
MAGdb = 20*log10(MAG);

bw_achieved = min(find(MAGdb<-3));      % - 3dB margin
bw_achieved = w(bw_achieved);

peak = max(MAGdb);


if bw_achieved < bw_min
    perf = 100e8;
    
else
    if max(find(ierr>SETTLE)) > 30      %% too slow response
        perf = 100e8;
    else
        perf1 = abs(bw_achieved - bw_target)/bw_target;         
        
        perf2 = abs(peak - peak_target)/peak_target;       % minimize resonant part
        
%         perf = max(find(ierr>SETTLE))/weight_speed;
        
        perf = perf1 + perf2; %abs(perf) + abs(perf1);
        
        %perf = sum((MAGdb - MAG_T2_db).^2);
    end
    
end
% if peak > 3
%     perf = 100e8;
% end