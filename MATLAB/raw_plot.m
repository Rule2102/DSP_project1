figure();
NOS = 16;
UR = 8;
NOS_UR = NOS/UR;
MAX_data_count = 650*NOS_UR;
data = zeros(MAX_data_count,3);

for i=1:1:3
s=num2str(i);
%f=strcat('E:\GIT\DSP_project1\MATLAB\dataOut_',s,'.dat');
f=strcat('D:\struka\DSP\project1\MATLAB\dataOut_',s,'.dat');
filename = f;
delimiter = ' ';
startRox = 2;
formatSpec = '%f%*s%*s%*s%*s%*s%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string', 'HeaderLines' ,startRox-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
hold all
data(:,i) = [dataArray{1:end-1}];
clearvars filename delimiter startRox formatSpec fileID dataArray ans;
hold all; stairs(data(:,i));
end

%% averaging
data1(:,1) = data(:,1);
data_avg = zeros(MAX_data_count);
for i=8:1:MAX_data_count-8
    data_avg(i,1) = sum(data1(i-7:1:i+8,1))/16;
end

figure();
stairs(data1(:,1)); 
hold all;
stairs(data_avg(:,1));

Ia = data_avg(:,1);
Ia = Ia(1:2:end);

data1(:,1) = data(:,2);
data_avg = zeros(MAX_data_count);
for i=8:1:MAX_data_count-8
    data_avg(i,1) = sum(data1(i-7:1:i+8,1))/16;
end

figure();
stairs(data1(:,1)); 
hold all;
stairs(data_avg(:,1));

Ib = data_avg(:,1);
Ib = Ib(1:2:end);

%%
theta = data(:,3);
theta = theta(1:2:end);
% Ia = data(:,1);
% Ib = data(:,2);

ADC_SCALE = 0.0007326;
LSB_offset_a = 2062;
ISENSE_OFFSET_A = LSB_offset_a*ADC_SCALE;
ISENSE_SCALE = 10;

Ia = (Ia*ADC_SCALE - ISENSE_OFFSET_A)*ISENSE_SCALE;
Ib = (Ib*ADC_SCALE - ISENSE_OFFSET_A)*ISENSE_SCALE;
Ialpha = Ia;
Ibeta = 0.57735*Ia + 1.1547*Ib;

sinth = sin(theta);
costh = cos(theta);

Id = Ialpha.*costh + Ibeta.*sinth;
Iq = Ibeta.*costh - Ialpha.*sinth;

figure();
stairs(Id);
hold all
stairs(Iq);
%%
Iqref = [7*ones(1,400),zeros(1,250)];
hold all; stairs(Iqref);
%%
tdsp = 1:1:650;
tdsp = tdsp - 401*ones(1,650);
tdsp= tdsp*100e-6/8;
figure();
stairs(tdsp,Id);
hold all
stairs(tdsp,Iq);
stairs(tdsp,Iqref);

alpha = 0.0636;
num = [4*alpha 0 0 0 0 0 0 0 0];
den = [4 -4 alpha 0 0 0 2*alpha 0 0 0 alpha];
Wcl= tf(num,den,100e-6/8);
opt = stepDataOptions('InputOffset',7,'StepAmplitude',-7);
step(tdsp,Wcl,opt)

%%

ADC_SCALE = 0.0007326;
LSB_offset_a = 2062;
ISENSE_OFFSET_A = LSB_offset_a*ADC_SCALE;
ISENSE_SCALE = 10;

Id = zeros(MAX_data_count,1);
Iq = zeros(MAX_data_count,1);
Ia = zeros(MAX_data_count,1);
Ib = zeros(MAX_data_count,1);
for i=NOS/2+1:NOS_UR:MAX_data_count-NOS/2
    for i2=1:1:UR
        is = i - (NOS/2-1) + NOS_UR*(i2-1);
        ie = is + (NOS_UR-1);
        Ia(i) = sum(data(is:ie,1))/NOS_UR;
        Ib(i) = sum(data(is:ie,2))/NOS_UR;
        isth = is-1;
        ieth = isth + NOS_UR;
        theta = (data(isth,3)+data(ieth,3))/2;
        
        Ia(i) = (Ia(i)*ADC_SCALE - ISENSE_OFFSET_A)*ISENSE_SCALE;
        Ib(i) = (Ib(i)*ADC_SCALE - ISENSE_OFFSET_A)*ISENSE_SCALE;
        Ialpha = Ia(i);
        Ibeta = 0.57735*Ia(i) + 1.1547*Ib(i);

        sinth = sin(theta);
        costh = cos(theta);

        Id(i) = Id(i) +Ialpha.*costh + Ibeta.*sinth;
        Iq(i) = Iq(i) + Ibeta.*costh - Ialpha.*sinth;
    end
    Id(i) = Id(i)/UR;
    Iq(i) = Iq(i)/UR;
end

figure();
stairs(Id(1:2:end));
hold all
stairs(Iq(1:2:end));

%% check for vertical crossings
load('saw.mat');
n_seg1 = data(:,3); % to sto ucitas jer je to pocetak narednog u kom se PWM_CMP primenjuje
n_seg = n_seg1(1,1);

n_start = n_seg*Ts/Ttbclk;
pwm_cmp_a = data(:,2);
saw_shift = pom(n_start:1:end);
figure();
tpom=0:Ttbclk:(length(saw_shift)-1)*Ttbclk;
stairs(tpom,saw_shift); % plot counter
hold all
tpom2 = 0:Ts:(length(pwm_cmp_a)-1)*Ts;
stairs(tpom2,pwm_cmp_a); % plot PWM_cmp

figure(); stairs(tpom2,data(:,1));

%plot for ur8 without MAF
% figure()
% pom1=zeros(1,212);
% pom2=zeros(1,212);
% for i=4:4:850
%     pom1(i/4) = data(i,1);
%     pom2(i/4) = data(i,2);
% end
% stairs(pom1);
% hold all
% stairs(pom2);

% for ADC offset measurement
% m1 = mean(data(:,1)) %-4094/2
% m2 = mean(data(:,2)) %-4094/2


%%
%import data from figure
fig = gcf;
axObjs = fig.Children;
dataObjs = axObjs.Children;
x = dataObjs(1).XData;
y = dataObjs(1).YData;

% make box and zoom
y1 = Iq;
x1 = 1:1:662;
%y2 = Iq;
axes('position',[.57 .29 .25 .25])
box on % put box around nex pair of axes
i1 = (x1 < 465) & (x1 > 435);
stairs(x1(i1),y(i1));
hold all
stairs(x1(i1),y2(i1));
axis tight
ylim([2.9 4.07]);

%%
figure()
% tpom = 1:1:2946;
% tpom2 = 1:4:2946;
Id = Idq(:,2);
Iq = Idq(:,3);
Id = Id(dmacnt_prnt+100:end);
Iq = Iq(dmacnt_prnt+100:end);
%t = 1:4:1674
hold all
stairs(Id);
stairs(Iq);
% stairs(tpom2,Id(end-736:end)); %stairs(tpom,Id(end-2945:end));
% stairs(tpom2,Iq(end-736:end));
%ylim([-1 5]);
%ylim([-0.3 2.3]);
%legend('Id_{HIL}','Iq_{HIL}','Id_{Simulink}','Iq_{Simulink}');
%title('IMC \alpha=0.15');
xlabel('Time [t/Ts]');
ylabel('Current [A]');
%title(['Benchmark: \alpha = ',num2str(alpha),' d = ',num2str(d)]);
%title(['UR=8 with MAF \alpha = ',num2str(alpha),' d = ',num2str(d),'no EMF']); %,'\newline','f_{bw}^{an}=3kHz, t_{rise}^{an}=1*T_{pwm}, t_{rise}^{sim}=1.1*T_{pwm}']);
%legend('Id ur2','Iq ur2','Id ur8','Iq ur8');
%legend('Id PI fb','Iq PI fb','Id PI err','Iq PI err');
%'Id PI_{error}','Iq PI_{error}','Id IMC','Iq IMC',
%legend('Id PI_{error}','Iq PI_{error}','Id IMC','Iq IMC','Id PI_{ff}','Iq PI_{ff}');

%% calculate thd

Ia = ia(:,2);
%Ia = Ia(end-1000000+1:end);
N = length(Ia);

ia_spectrum_c = fftshift(fft(Ia))/(N/2);

N1 = 30; % first N1 harmonics are of interest
f2 = linspace(1,N1,N1); % frequenxcy vector used for plotting

ia_spectrum_c_shift = zeros(1,N1);
for p=1:N1
 ia_spectrum_c_shift(p) = ia_spectrum_c(N/2+1+3*p);
end

ia_spectrum_abs = abs(ia_spectrum_c_shift);
%
figure();
bar(f2,ia_spectrum_abs);
%}

sum_harm = 0;
for p=2:N1
    sum_harm = sum_harm + (ia_spectrum_abs(p)/sqrt(2))^2;
end

% if(sum_harm < 1e-5) 
%     sum_harm = 0;
% end

THD = sqrt(sum_harm)/(ia_spectrum_abs(1)/sqrt(2))*100

if (OVERSAMPLING == 1) 
    pom = 'with';
else
    pom = 'no';
end

if (f_ref == 100) 
    pom2 = '0.25<d<0.75';
else
    pom2 = '0.15<d<0.85';
end

title(['Phase current spectrum UR=8 ',pom,' MAF, ',pom2,': THD = ',num2str(THD)]);


%% plot duty cycle (check for vertical crossings)
dc_t = vert.time;
dc_d = vert.signals(1).values;
dc_a = dc_d(:,1);
dc_b = dc_d(:,2);
dc_c = dc_d(:,3);
dc_car = vert.signals(2).values;

figure()
stairs(dc_t,dc_a);
hold all
stairs(dc_t,dc_b);
stairs(dc_t,dc_c);
stairs(dc_t,dc_car);
legend('da','db','dc','carrier');
title(['Modulation signals UR=8 ',pom,' MAF, ',pom2]);

%%
pom=zeros(800,1);
for i=1:1:800
    pom(i) = data(i,1)+data(i,2);
    if(pom(i) >= 2*pi) pom(i)=pom(i)-2*pi;
    else if (pom(i)<0) pom(i)=pom(i)+2*pi;
        end
    end
end
%%
w=2*pi;
k=1;
s=tf('s');
Gd = k*w*s/(s^2+k*w*s+w^2);
bode(Gd);
hold all;
Gq = k*w^2/(s^2+k*w*s+w^2);
bode(Gq)
%%

csaw=saw(:,2);
n1 = 800*Ts/Tsim+1;
n2 = n1 + 250*Ts/Tsim;
tsaw2 = 0:1:250*Ts/Tsim;
csaw2 = csaw(n1:n2);
figure();
stairs(tsaw2,csaw2);

%%
tcmp2 = 0:Ts/Tsim:249*Ts/Tsim;
cmpreg2 = cmpreg(:,2);
cmpreg2 = cmpreg2(801:1050);
hold all;
stairs(tcmp2,cmpreg2);

%%

Id = Idq(:,3);
pom1 = round(dmacnt_ref*0.9);
pom2 = round(dmacnt_ref*1.1);
Id = Id(pom1+1:pom2);
figure();
stairs(Id);
ylim([0 10]);

%%


H = tf([-1.2,-2.4,-1.5],[1,20,9.1]);
w = logspace(-2,3,101);

sysestim = frd(H,w);
save('test.mat','sysestim');










