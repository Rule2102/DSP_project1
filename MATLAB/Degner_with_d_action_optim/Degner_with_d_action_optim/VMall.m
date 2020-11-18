% VM = min[ abs(1/(1+Wpp)) ] za pun opseg ucestanosti

close all; 
alfa = exp(-0.47*0.00005/0.0033); Raggio = 1000; 

NS = size(numpp(:),1);    % Size of numerator 
DS = size(denpp(:),1);    % Size of denominator

MIN = 500; MAX = 9500;
    for jj=MIN:MAX,
      We=(jj/10000)*2*pi; zz = exp(1i*We); 
      
      Broj = 0; vz = 1; 
      for qq = 1:NS,
          Broj = Broj + numpp(NS-qq+1) * vz;
          vz = vz * zz; 
      end;
      
      Imen = 0; vz = 1; 
      for qq = 1:DS,
          Imen = Imen + denpp(DS-qq+1) * vz;
          vz = vz * zz; 
      end
      
% --- Izracunavanje WPP --- Begin ---
      
      WPP = Broj/Imen; 
% --- Izracunavanje WPP --- End ---

      GGG = 1/(1+WPP); AMP(jj)=abs(GGG);
      RR(jj) = real(WPP); II(jj) = imag(WPP); 
      Tacka = (-1 + 0i);  Radwpp = WPP - Tacka; Radius = abs(Radwpp);
      if Radius < Raggio, Raggio = Radius; end; 
      end; 
plot(RR(MIN:MAX),II(MIN:MAX)); axis([-1.2 0.8 -1 1]); grid; 
VM=1/max(AMP);
