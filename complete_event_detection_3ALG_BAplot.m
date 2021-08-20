%% COMPLETE EVENT DETECTION IMU-FS  
% 3ALG : detection based on Gyr Z and threshold

clear all;
close all;
clc;

%% EDITABLE PARAMETERS
% note: start detection at 5 s (before beta=0.6)
% stopind_fs=256000;
% stopind_imu=12800;

load('stopind');

% FOOT SWITCH (Gold Standard) PARAMTERS
thr_gs=2;                           % threshold (magnitude) for event detection
thrTime_gs=0.4;                     % threshold (time) - (does not count peaks closer than x seconds)

% start detection FS after 10 s
ind_heelRight=20000;                % start event detection for FS from sample ind_gs
ind_toeRight=20000;
ind_heelLeft=20000;
ind_toeLeft=20000;

% start detection IMU after 10 s
ind_R=1000;                         % start event detection for IMU from sample ind_R or ind_L
ind_L=1000;

%% IMU PARAMETERS - SLOW WALK
% THRESHOLDS for R leg
thrR_TO=50;                        % Threshold to detect Toe Off point (samples)
thrR_HS=-37;
thrR_MS=127;

% THRESHOLDS for L leg (dovrebbero andare bene per tutte le acquisizioni
thrL_TO=50;                         % Threshold to detect Toe Off point (samples)
thrL_HS=-37;
thrL_MS=127;

%% IMU PARAMETERS - FAST WALK
% THRESHOLDS for R leg
thrR_TO=50;                        % Threshold to detect Toe Off point (samples)
thrR_HS=-60;
thrR_MS=200;

% THRESHOLDS for L leg (dovrebbero andare bene per tutte le acquisizioni
thrL_TO=50;                         % Threshold to detect Toe Off point (samples)
thrL_HS=-60;
thrL_MS=200;

%% IMU PARAMETERS - STIFF WALK
% THRESHOLDS for R leg
thrR_TO=50;                        % Threshold to detect Toe Off point (samples)
thrR_HS=-15;
thrR_MS=40;

% THRESHOLDS for L leg (dovrebbero andare bene per tutte le acquisizioni
thrL_TO=50;                         % Threshold to detect Toe Off point (samples)
thrL_HS=-15;
thrL_MS=90;

%% LOAD DATA from FOOT SWITCH 

% open file "syn_P*GE**"

fs_data = 2000; 
nF_data = size(new_data,1);                         %number of row
t_data = [0:nF_data-1]'/fs_data;                    %time array

% (P8,P7,P6,P5,P2,P1) 
TL_FS = new_data(:,1);
HL_FS = new_data(:,2);
TR_FS = new_data(:,3);
HR_FS = new_data(:,4);

% (P4,P3)
% TL_FS = new_data(:,2);
% HL_FS = new_data(:,1);
% TR_FS = new_data(:,4);
% HR_FS = new_data(:,3);

% MSL_FS = new_data(:,5);
% MSR_FS = -new_data(:,6);

%% LOAD DATA from IMU SENSOR L/R SHANK

Matrix_2 = readmatrix('Raw IMU Data - ShankL.csv');                      % left shank
Matrix_3 = readmatrix('Raw IMU Data - ShankR.csv');                      % right shank

acc_LS = Matrix_2(:,[1 2 3]);                                       % left shank accelerometer data
gyr_LS = Matrix_2(:,[4 5 6]);                                       % left shank gyroscope data

acc_RS  = Matrix_3(:,[1 2 3]);                                       % right shank accelerometer data
gyr_RS  = Matrix_3(:,[4 5 6]);                                       % right shank gyroscope data

clear Matrix_2 Matrix_3;

% sampling frequency and time array
fs = 100;                                 %sampling rate
nF = size(acc_RS,1);                      %number of row
t = [0:nF-1]'/fs;                         %time array
deltaT = 1/fs;                            %sampling time

%% segment orientation 
% PROVA NUOVO FILTRO DI MADGWICK

AHRS = MadgwickAHRS('SamplePeriod', 1/fs);


q_LS = zeros(length(t), 4);
for i = 1:length(t)
    AHRS.UpdateIMU(gyr_LS(i,:) * (pi/180), acc_LS(i,:), i);	% gyroscope units must be radians
    q_LS(i,:) = AHRS.Quaternion;
end

q_RS = zeros(length(t), 4);
for i = 1:length(t)
    AHRS.UpdateIMU(gyr_RS(i,:) * (pi/180), acc_RS(i,:), i);	% gyroscope units must be radians
    q_RS(i,:) = AHRS.Quaternion;
end


% euler angles from quaternion
ang_LS = quatern2euler(quaternConj(q_LS)) * (180/pi);
ang_RS = quatern2euler(quaternConj(q_RS)) * (180/pi);

% flip data for left leg
ang_LS = -ang_LS;
gyr_LS = -gyr_LS;

%(verify) plot displacement on saggital plane 
figure(7)
hold on
title('left shank displacement (blue) and right shank displacement (red)')
plot(t,ang_LS(:,2),'b',t,ang_RS(:,2),'r')
hold off


%% IMU Signals

% RIGHT
gyrZ_RS = gyr_RS(:,3);
angY_RS = ang_RS(:,2);


% LEFT
gyrZ_LS = gyr_LS(:,3);
angY_LS = ang_LS(:,2);


%% check synchronization

figure(11)
title('Synchronization test')
hold on
plot(t_data,HR_FS*25,'b')
plot(t,gyrZ_RS,'r')
% plot(t,acc_filt_RS*100,'k')  %,'Acc magnitude filtered'
legend('Heel foot switch','Gyr Z')
xlabel('Time (s)')


% figure(12)
% title('Synchronization test Left')
% hold on
% plot(t_data,HL_FS*25,'b')
% plot(t,gyrZ_LS,'r')
% % plot(t,acc_filt_RS*100,'k')  %,'Acc magnitude filtered'
% legend('Heel foot switch','Gyr Z')
% xlabel('Time (s)')

figure(13)
sgtitle('footswitch timing R and L + gyr Data')
subplot(2,1,1)
hold on
plot(t_data,HR_FS)
plot(t_data,TR_FS)
plot(t,gyrZ_RS/100)
xlabel('Time (s)')
ylabel('Magnitude')
legend('right heel','right toe','right gyrZ')
xlim([40 60])
hold off
subplot(2,1,2)
hold on
plot(t_data,HL_FS)
plot(t_data,TL_FS)
plot(t,gyrZ_LS/100)
xlim([40 60])
xlabel('Time (s)')
ylabel('Magnitude')
legend('left heel','left toe','left gyrZ')
hold off


figure(14)
plot(t,gyrZ_RS,t,gyrZ_LS)
xlabel('Time (s)')
ylabel('Magnitude')
legend('Right gyr Z','left gyr Z')


%% derivative FS data

% derivata 1
D1_TL_FS = gradient(TL_FS);
D1_HL_FS = gradient(HL_FS);
D1_TR_FS = gradient(TR_FS);
D1_HR_FS = gradient(HR_FS);

% derivata 2
D2_TL_FS = gradient(D1_TL_FS);
D2_HL_FS = gradient(D1_HL_FS);
D2_TR_FS = gradient(D1_TR_FS);
D2_HR_FS = gradient(D1_HR_FS);


%% left heel strike

heelRight = zeros(length(t_data),1);
time_heelRight=zeros(length(t_data),1);


for i=ind_heelRight:stopind_fs
    for ii=ind_heelRight:stopind_fs
        if (D1_HR_FS(ii-1)>thr_gs) && (D1_HR_FS(ii-1)>D1_HR_FS(ii)) && (D2_HR_FS(ii-1)<0)
            heelRight(ii-1)=1;
            time_heelRight(ii-1)=t_data(ii-1);
            break;
        end
    end
    if heelRight(ii-1)==1
        for k=ii:stopind_fs
            if (D1_HR_FS(k-1)==0) 
               ind_heelRight=k; 
               break;
            end 
        end
    end  
end

vectnz_heelRight=nonzeros(heelRight);
vectnz_time_heelRight=nonzeros(time_heelRight);

% indice che conta gli eventi di heel strike
count_heelRight=1;
% vettore che memorizza i tempi in cui sono individuati gli HS corretti
tempoeventicorretti_HRfs=zeros(length(vectnz_heelRight),1);
% questo vettore ha solo il primo valore temporale del primo HS individuato
% e poi tanti zeri quanti la lunghezza del vettore vectnz_heelLeft
for w=1:length(vectnz_heelRight)
    if w==1
        tempoeventicorretti_HRfs(w)=vectnz_time_heelRight(w);
    end
end
% ciclo che conta gli eventi rilevati scludendo dalla conta quelli più vicini
% di thr_time, e per ogni evento corretto memorizza il valore temporale.
for j=2:length(vectnz_time_heelRight)
    if vectnz_time_heelRight(j)-vectnz_time_heelRight(j-1)>thrTime_gs
        count_heelRight=count_heelRight+1;
        tempoeventicorretti_HRfs(j)=vectnz_time_heelRight(j);
    end
end
tempoeventicorretti_HRfs=nonzeros(tempoeventicorretti_HRfs);


%% right toe off

toeRight = zeros(length(t_data),1);
time_toeRight=zeros(length(t_data),1);


for i=ind_toeRight:stopind_fs
    for ii=ind_toeRight:stopind_fs
        if (D1_TR_FS(ii-1)<-thr_gs) && (D1_TR_FS(ii-1)<D1_TR_FS(ii)) && (D2_TR_FS(ii-1)>0)
            toeRight(ii-1)=1;
            time_toeRight(ii-1)=t_data(ii-1);
            break;
        end
    end
    if toeRight(ii-1)==1
        for k=ii:stopind_fs
            if (D1_TR_FS(k-1)==0) 
               ind_toeRight=k; 
               break;
            end 
        end
    end  
end

vectnz_toeRight=nonzeros(toeRight);
vectnz_time_toeRight=nonzeros(time_toeRight);

% indice che conta gli eventi di toe off
count_toeRight=1;
% vettore che memorizza i tempi in cui sono individuati gli HS corretti
tempoeventicorretti_TRfs=zeros(length(vectnz_toeRight),1);
% questo vettore ha solo il primo valore temporale del primo HS individuato
% e poi tanti zeri quanti la lunghezza del vettore vectnz_heelRight
for w=1:length(vectnz_toeRight)
    if w==1
        tempoeventicorretti_TRfs(w)=vectnz_time_toeRight(w);
        
    end
end

% ciclo che conta gli eventi rilevati scludendo dalla conta quelli più vicini
% di thr_time_gs, e per ogni evento corretto memorizza il valore temporale.
for j=2:length(vectnz_time_toeRight)
    if vectnz_time_toeRight(j)-vectnz_time_toeRight(j-1)>thrTime_gs
        count_toeRight=count_toeRight+1;
        tempoeventicorretti_TRfs(j)=vectnz_time_toeRight(j);
    end
end
tempoeventicorretti_TRfs=nonzeros(tempoeventicorretti_TRfs);


%% left heel strike

heelLeft = zeros(length(t_data),1);
time_heelLeft=zeros(length(t_data),1);


for i=ind_heelLeft:stopind_fs
    for ii=ind_heelLeft:stopind_fs
        if (D1_HL_FS(ii-1)>thr_gs) && (D1_HL_FS(ii-1)>D1_HL_FS(ii)) && (D2_HL_FS(ii-1)<0)
            heelLeft(ii-1)=1;
            time_heelLeft(ii-1)=t_data(ii-1);
            break;
        end
    end
    if heelLeft(ii-1)==1
        for k=ii:stopind_fs
            if (D1_HL_FS(k-1)==0) 
               ind_heelLeft=k; 
               break;
            end 
        end
    end  
end

vectnz_heelLeft=nonzeros(heelLeft);
vectnz_time_heelLeft=nonzeros(time_heelLeft);

% indice che conta gli eventi di heel strike
count_heelLeft=1;
% vettore che memorizza i tempi in cui sono individuati gli HS corretti
tempoeventicorretti_HLfs=zeros(length(vectnz_heelLeft),1);
% questo vettore ha solo il primo valore temporale del primo HS individuato
% e poi tanti zeri quanti la lunghezza del vettore vectnz_heelLeft
for w=1:length(vectnz_heelLeft)
    if w==1
        tempoeventicorretti_HLfs(w)=vectnz_time_heelLeft(w);
    end
end
% ciclo che conta gli eventi rilevati scludendo dalla conta quelli più vicini
% di thr_time, e per ogni evento corretto memorizza il valore temporale.
for j=2:length(vectnz_time_heelLeft)
    if vectnz_time_heelLeft(j)-vectnz_time_heelLeft(j-1)>thrTime_gs
        count_heelLeft=count_heelLeft+1;
        tempoeventicorretti_HLfs(j)=vectnz_time_heelLeft(j);
    end
end
tempoeventicorretti_HLfs=nonzeros(tempoeventicorretti_HLfs);



%% left toe off

toeLeft = zeros(length(t_data),1);
time_toeLeft=zeros(length(t_data),1);


for i=ind_toeLeft:stopind_fs
    for ii=ind_toeLeft:stopind_fs
        if (D1_TL_FS(ii-1)<-thr_gs) && (D1_TL_FS(ii-1)<D1_TL_FS(ii)) && (D2_TL_FS(ii-1)>0)
            toeLeft(ii-1)=1;
            time_toeLeft(ii-1)=t_data(ii-1);
            break;
        end
    end
    if toeLeft(ii-1)==1
        for k=ii:stopind_fs
            if (D1_TL_FS(k-1)==0) 
               ind_toeLeft=k; 
               break;
            end 
        end
    end  
end

vectnz_toeLeft=nonzeros(toeLeft);
vectnz_time_toeLeft=nonzeros(time_toeLeft);

% indice che conta gli eventi di toe off
count_toeLeft=1;
% vettore che memorizza i tempi in cui sono individuati gli HS corretti
tempoeventicorretti_TLfs=zeros(length(vectnz_toeLeft),1);
% questo vettore ha solo il primo valore temporale del primo HS individuato
% e poi tanti zeri quanti la lunghezza del vettore vectnz_heelRight
for w=1:length(vectnz_toeLeft)
    if w==1
        tempoeventicorretti_TLfs(w)=vectnz_time_toeLeft(w);
        
    end
end

% ciclo che conta gli eventi rilevati scludendo dalla conta quelli più vicini
% di thr_time_gs, e per ogni evento corretto memorizza il valore temporale.
for j=2:length(vectnz_time_toeLeft)
    if vectnz_time_toeLeft(j)-vectnz_time_toeLeft(j-1)>thrTime_gs
        count_toeLeft=count_toeLeft+1;
        tempoeventicorretti_TLfs(j)=vectnz_time_toeLeft(j);
    end
end
tempoeventicorretti_TLfs=nonzeros(tempoeventicorretti_TLfs);

%% plot fs event
figure(1)
sgtitle('Foot switch data')
subplot(2,2,1)
plot(t_data,HR_FS)
hold on
plot(t_data,heelRight,'rx')
hold off
xlabel('Time (s)')
ylabel('Heel strike left detected (x)')

subplot(2,2,2)
plot(t_data,TR_FS)
hold on
plot(t_data,toeRight,'rx')
hold off
xlabel('Time (s)')
ylabel('Toe off right detected (x)')

subplot(2,2,3)
plot(t_data,HL_FS)
hold on
plot(t_data,heelLeft,'rx')
hold off
xlabel('Time (s)')
ylabel('Heel strike left detected (x)')

subplot(2,2,4)
plot(t_data,TL_FS)
hold on
plot(t_data,toeLeft,'rx')
hold off
xlabel('Time (s)')
ylabel('Toe off left detected (x)')


%% Detect HS,TO,MS (with thr) + TEMPORAL PARAMETERS - R/L leg

% STRIDE TIME=HS(i)-HS(i-1)
% SWING TIME=HS(i)-TO(i-1)
% STANCE TIME=TO(i)-HS(i)

% x(i)= input sample
% x(i-1)= previous sample
% x(i-2)= 2 previous sample --> this introduct a delay >=0.03 s (30 ms)

% Initialize array for R leg
HS_R=zeros(length(t),1);        % Heel Strike
time_HSR=zeros(length(t),1);
TO_R=zeros(length(t),1);        % Toe off
time_TOR=zeros(length(t),1);
MS_R=zeros(length(t),1);        % Mid Swing


% initialize the indexes for R leg
indHS_R=0;
indTO_R=0;
indMS_R=0;
t_HSR_pre=0;             % Time of previous HS
t_TOR_pre=0;             % Time of previous TO 


for jj=ind_R:stopind_imu
    
    % look for Heel Strike point (HS_R = 1) && (angY_RS(p-1)<angY_RS(p))
    for p=ind_R:stopind_imu
        if  (gyrZ_RS(p-1)<gyrZ_RS(p)) && (gyrZ_RS(p-1)<thrR_HS) 
            HS_R(p-1)=1;
            time_HSR(p-1)=t(p-1);
            indHS_R=p;                              % update index
            swing_time_R=t(p-1)-t_TOR_pre;           % SWING TIME of previous step
            stride_time_R=t(p-1)-t_HSR_pre;          % STRIDE TIME of previous step           
            t_HSR_pre=t(p-1);                       % update index
            break
        else
            HS_R(p-1)=0;
        end
    end
      
    % look for Toe off point (TO_R = 1) && (angY_RS(k-1)<angY_RS(k))    
    if HS_R(p-1)==1
        for k=indHS_R+thrR_TO:stopind_imu
            if (gyrZ_RS(k-1)<gyrZ_RS(k)) && (gyrZ_RS(k-1)<=0) && (gyrZ_RS(k)>0)
                TO_R(k-1)=1;
                time_TOR(k-1)=t(k-1);
                indTO_R=k;                              % update index
                stance_time_R=t(k-1)-t_HSR_pre;          % STANCE TIME of step "in time"
                t_TOR_pre=t(k-1);                       % update index                
                break
            else
                TO_R(k-1)=0;
            end
        end
    end 
    
    % look for Mid swing point (MS_R = 1)
    if TO_R(k-1)==1
        for j=indTO_R:stopind_imu
            if (angY_RS(j-1)>angY_RS(j)) && (gyrZ_RS(j-1)>gyrZ_RS(j)) && (gyrZ_RS(j-1)>thrR_MS)
                MS_R(j-1)=1;
                indMS_R=j;
                ind_R=indMS_R;                        % update principal index
                break
            else
                MS_R(j-1)=0;
            end
        end
    end

    
end
    
% plot 
figure(2)
plot(t,gyrZ_RS/100,'Color','#EDB120')
hold on
plot(t,HS_R,'xr','LineWidth',2)
plot(t,TO_R,'x','Color','#77AC30','LineWidth',2)
plot(t,MS_R,'x','Color','#7E2F8E','LineWidth',2)
hold off
ylim([0 2.3])
xlim([0 90])
xlabel('Time (s)')
Leg=legend('Gyr Z','HS point','TO point','MS point');
Leg.FontSize=14;
Tle=title('HS, TO and MS detection (right leg)',['Threshold (samples) for TO=',num2str(thrR_TO)]);
Tle.FontSize=14;

% conto gli eventi di to e hs identificati con le IMU
vectnz_imurhs=nonzeros(HS_R);
vectnz_time_imurhs=nonzeros(time_HSR);
vectnz_imurto=nonzeros(TO_R);
vectnz_time_imurto=nonzeros(time_TOR);
count_ImuHSR=0;
count_ImuTOR=0;

tempoeventicorretti_HSR=zeros(length(vectnz_imurhs),1);
tempoeventicorretti_TOR=zeros(length(vectnz_imurto),1);
for j=1:length(vectnz_imurhs)
    if vectnz_imurhs ==1
        count_ImuHSR=count_ImuHSR+1;
        tempoeventicorretti_HSR(j)=vectnz_time_imurhs(j);
    end
end
for j=1:length(vectnz_imurto)
    if vectnz_imurto ==1
        count_ImuTOR=count_ImuTOR+1;
        tempoeventicorretti_TOR(j)=vectnz_time_imurto(j);
    end
end

% Initialize array for L leg
HS_L=zeros(length(t),1);        % Heel Strike
time_HSL=zeros(length(t),1);
TO_L=zeros(length(t),1);        % Toe off
time_TOL=zeros(length(t),1);
MS_L=zeros(length(t),1);        % Mid Swing


% initialize the indexes for L leg
indHS_L=0;
indTO_L=0;
indMS_L=0;
t_HSL_pre=0;             % Time of previous HS
t_TOL_pre=0;             % Time of previous TO 

 
for JJ=ind_L:stopind_imu
    
    % look for Heel strike point (HS_L = 1) && (angY_LS(P-1)<angY_LS(P))
    for P=ind_L:stopind_imu
        if  (gyrZ_LS(P-1)<gyrZ_LS(P)) && (gyrZ_LS(P-1)<thrL_HS) 
            HS_L(P-1)=1;
            time_HSL(P-1)=t(P-1);
            indHS_L=P;                              % update index
            swing_time_L=t(P-1)-t_TOL_pre;           % SWING TIME  of previous step
            stride_time_L=t(P-1)-t_HSL_pre;          % STRIDE TIME of previous step            
            t_HSL_pre=t(P-1);                       % update index            
            break
        else
            HS_L(P-1)=0;
        end
    end
    
    % look for Toe off point (TO_L = 1) && (angY_LS(K-1)<angY_LS(K)) 
    if HS_L(P-1)==1
        for K=indHS_L+thrL_TO:stopind_imu
            if (gyrZ_LS(K-1)<gyrZ_LS(K)) && (gyrZ_LS(K-1)<=0) && (gyrZ_LS(K)>0)
                TO_L(K-1)=1;
                time_TOL(K-1)=t(K-1);
                indTO_L=K;                              % update index
                stance_time_L=t(K-1)-t_HSL_pre;          % STANCE TIME of step "in time"
                t_TOL_pre=t(K-1);                       % update index
                break
            else
                TO_L(K-1)=0;
            end
        end
    end
    
    % look for Mid swing point (MS_L = 1)
    if TO_L(K-1)==1
        for J=indTO_L:stopind_imu
            if (angY_LS(J-1)>angY_LS(J)) && (gyrZ_LS(J-1)>gyrZ_LS(J)) && (gyrZ_LS(J-1)>thrL_MS)
                MS_L(J-1)=1;
                indMS_L=J;                              % update principal index
                ind_L=indMS_L;
                break
            else
                MS_L(J-1)=0;
            end
        end
    end

end



% plot 
figure(3)
plot(t,gyrZ_LS/100,'Color','#EDB120')
hold on
plot(t,HS_L,'xr','LineWidth',2)
plot(t,TO_L,'x','Color','#77AC30','LineWidth',2)
plot(t,MS_L,'x','Color','#7E2F8E','LineWidth',2)
hold off
ylim([0 2.3])
xlim([0 90])
xlabel('Time (s)')
Leg=legend('Gyr Z','HS point','TO point','MS point')
Leg.FontSize=14;
Tle=title('HS, TO and MS detection (left leg)',['Threshold (samples) for TO=',num2str(thrR_TO)])
Tle.FontSize=14


% conto gli eventi di to e hs identificati con le IMU
vectnz_imulhs=nonzeros(HS_L);
vectnz_time_imulhs=nonzeros(time_HSL);
vectnz_imulto=nonzeros(TO_L);
vectnz_time_imulto=nonzeros(time_TOL);
count_ImuHSL=0;
count_ImuTOL=0;

tempoeventicorretti_HSL=zeros(length(vectnz_imulhs),1);
tempoeventicorretti_TOL=zeros(length(vectnz_imulto),1);
for j=1:length(vectnz_imulhs)
    if vectnz_imulhs ==1
        count_ImuHSL=count_ImuHSL+1;
        tempoeventicorretti_HSL(j)=vectnz_time_imulhs(j);
    end
end
for j=1:length(vectnz_imulto)
    if vectnz_imulto ==1
        count_ImuTOL=count_ImuTOL+1;
        tempoeventicorretti_TOL(j)=vectnz_time_imulto(j);
    end
end


%% compare event plot

figure(4)
sgtitle('Quantitative comparison of events detected')
subplot(4,1,1)
plot(t_data,heelRight,'ro')
hold on
plot(t,HS_R,'bx')
legend(['Foot switch =',num2str(count_heelRight)],['IMU =',num2str(count_ImuHSR)])
hold off
ylim([0 2])
title('Event detected - Heel strike right','FontSize',12)
xlabel('Time (s)')

subplot(4,1,2)
plot(t_data,toeRight,'ro')
hold on
plot(t,TO_R,'bx')
legend(['Foot switch =',num2str(count_toeRight)],['IMU =',num2str(count_ImuTOR)])
hold off
ylim([0 2])
title('Event detected - Toe off right','FontSize',12)
xlabel('Time (s)')

subplot(4,1,3)
plot(t_data,heelLeft,'ro')
hold on
plot(t,HS_L,'bx')
legend(['Foot switch =',num2str(count_heelLeft)],['IMU =',num2str(count_ImuHSL)])
hold off
ylim([0 2])
title('Event detected - Heel strike left','FontSize',12)
xlabel('Time (s)')

subplot(4,1,4)
plot(t_data,toeLeft,'ro')
hold on
plot(t,TO_L,'bx')
legend(['Foot switch =',num2str(count_toeLeft)],['IMU =',num2str(count_ImuTOL)])
hold off
ylim([0 2])
title('Event detected - Toe off left','FontSize',12)
xlabel('Time (s)')

%% BLAND-ALTMAN PLOT
% data_mean= (data_fs+data_imu)/2
% data_diff= data_imu-data_fs
% md= mean(data_diff)
% sd= sd(data_diff)


% cut data manually
% tempoeventicorretti_HRfs=tempoeventicorretti_HRfs([2:end],:);
% tempoeventicorretti_HSR=tempoeventicorretti_HSR([2:end],:);
% 
% tempoeventicorretti_TRfs=tempoeventicorretti_TRfs([2:end],:);
% tempoeventicorretti_TOR=tempoeventicorretti_TOR([2:end],:);

% tempoeventicorretti_HLfs=tempoeventicorretti_HLfs([2:end],:);
% tempoeventicorretti_HSL=tempoeventicorretti_HSL([2:end],:);

% tempoeventicorretti_TLfs=tempoeventicorretti_TLfs([1:end],:);
% tempoeventicorretti_TOL=tempoeventicorretti_TOL([1:end],:);


% R HEEL STRIKE - TOE OFF
[data_mean_RHS,data_diff_RHS,md_RHS,sd_RHS] = bland_altman(tempoeventicorretti_HRfs,tempoeventicorretti_HSR);
[data_mean_RTO,data_diff_RTO,md_RTO,sd_RTO] = bland_altman(tempoeventicorretti_TRfs,tempoeventicorretti_TOR);
% R/L TOE OFF
[data_mean_LHS,data_diff_LHS,md_LHS,sd_LHS] = bland_altman(tempoeventicorretti_HLfs,tempoeventicorretti_HSL);
[data_mean_LTO,data_diff_LTO,md_LTO,sd_LTO] = bland_altman(tempoeventicorretti_TLfs,tempoeventicorretti_TOL);

% 
figure(5)
sgtitle('Bland-Altman plot: Mean = (data IMU + data FS)/2 ; Difference = data IMU - data FS')
subplot(2,2,1)
hold on
plot(data_mean_RHS,data_diff_RHS,'or','MarkerSize',8,'LineWidth',1.5)
yline(md_RHS,'-k','LineWidth',1.5)
yline(md_RHS+(1.96*sd_RHS),'--b',['mean +(1.96 SD)  =',(num2str(md_RHS+(1.96*sd_RHS)))],'LineWidth',1.5)
yline(md_RHS-(1.96*sd_RHS),'--b',['mean -(1.96 SD)  =',(num2str(md_RHS-(1.96*sd_RHS)))],'LineWidth',1.5)
title(['Right Heel Strike, sample = ', (num2str(length(tempoeventicorretti_HRfs)))],'FontSize',14)
subtitle(['(Mean±SD) = ',(num2str(md_RHS)),'±',(num2str(sd_RHS))],'FontSize',12)
xlabel('Mean','FontSize',12)
ylabel('Difference','FontSize',12)
hold off

subplot(2,2,2)
hold on
plot(data_mean_RTO,data_diff_RTO,'or','MarkerSize',8,'LineWidth',1.5)               % Bland Altman plot
yline(md_RTO,'-k','LineWidth',1.5)
yline(md_RTO+(1.96*sd_RTO),'--b',['mean +(1.96 SD)  =',(num2str(md_RTO+(1.96*sd_RTO)))],'LineWidth',1.5)
yline(md_RTO-(1.96*sd_RTO),'--b',['mean -(1.96 SD) =',(num2str(md_RTO-(1.96*sd_RTO)))],'LineWidth',1.5)
title(['Right Toe Off, sample = ',(num2str(length(tempoeventicorretti_TRfs)))],'FontSize',14)
subtitle(['(Mean±SD) = ',(num2str(md_RTO)),'±',(num2str(sd_RTO))],'FontSize',12)
xlabel('Mean','FontSize',12)
ylabel('Difference','FontSize',12)
hold off

subplot(2,2,3)
hold on
plot(data_mean_LHS,data_diff_LHS,'or','MarkerSize',8,'LineWidth',1.5)              
yline(md_LHS,'-k','LineWidth',1.5)
yline(md_LHS+(1.96*sd_LHS),'--b',['mean +(1.96 SD) =',(num2str(md_LHS+(1.96*sd_LHS)))],'LineWidth',1.5)
yline(md_LHS-(1.96*sd_LHS),'--b',['mean-(1.96 SD) =',(num2str(md_LHS-(1.96*sd_LHS)))],'LineWidth',1.5)
title(['Left Heel Strike, sample = ', (num2str(length(tempoeventicorretti_HLfs)))],'FontSize',14)
subtitle(['(Mean±SD) = ',(num2str(md_LHS)),'±',(num2str(sd_LHS))],'FontSize',12)
xlabel('Mean','FontSize',12)
ylabel('Difference','FontSize',12)
hold off

subplot(2,2,4)
hold on
plot(data_mean_LTO,data_diff_LTO,'or','MarkerSize',8,'LineWidth',1.5)
yline(md_LTO,'-k','LineWidth',1.5)
yline(md_LTO+(1.96*sd_LTO),'--b',['mean +(1.96 SD)  =',(num2str(md_LTO+(1.96*sd_LTO)))],'LineWidth',1.5)
yline(md_LTO-(1.96*sd_LTO),'--b',['mean -(1.96 SD) =',(num2str(md_LTO-(1.96*sd_LTO)))],'LineWidth',1.5)
title(['Left Toe Off, sample = ',(num2str(length(tempoeventicorretti_TLfs)))],'FontSize',14)
subtitle(['(Mean±SD) = ',(num2str(md_LTO)),'±',(num2str(sd_LTO))],'FontSize',12)
xlabel('Mean','FontSize',12)
ylabel('Difference','FontSize',12)
hold off


