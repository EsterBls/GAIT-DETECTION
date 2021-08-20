%% COMPLETE EVENT DETECTION IMU  
% 3ALG : detection based on Gyr Z and threshold

% 20-08-2021

  
    
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



%% SEGMENT ORIENTATION 
% Madgwick filter 

% FAST WALK
AHRS = MadgwickAHRS('SamplePeriod', 1/fs,'Beta1',0.6,'Beta2',0.6);

% SLOW WALK, SLOW AND STIFF WALK
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


    
%% SECTION I: DETECT HS,TO,MS (with thr) + TEMPORAL PARAMETERS - R/L leg

%% EDITABLE PARAMETERS
% note: start detection at 10 s (before beta=0.6)
ind_R=1000;                         % start event detection for IMU from sample ind_R or ind_L
ind_L=1000;

% stop detection index
load('stopind');

%% IMU PARAMETERS - SLOW WALK
% THRESHOLDS for R leg
thrR_TO=50;                        % Threshold to detect Toe Off point (samples)
thrR_HS=-37;                       % Amplitude threshold
thrR_MS=127;                       % Amplitude threshold

% THRESHOLDS for L leg 
thrL_TO=50;                         % Threshold to detect Toe Off point (samples)
thrL_HS=-37;
thrL_MS=127;

%% IMU PARAMETERS - FAST WALK
% THRESHOLDS for R leg
thrR_TO=50;                        % Threshold to detect Toe Off point (samples)
thrR_HS=-60;
thrR_MS=200;

% THRESHOLDS for L leg 
thrL_TO=50;                         % Threshold to detect Toe Off point (samples)
thrL_HS=-60;
thrL_MS=200;

%% IMU PARAMETERS - STIFF WALK
% THRESHOLDS for R leg
thrR_TO=50;                        % Threshold to detect Toe Off point (samples)
thrR_HS=-15;
thrR_MS=40;

% THRESHOLDS for L leg 
thrL_TO=50;                         % Threshold to detect Toe Off point (samples)
thrL_HS=-15;
thrL_MS=90;



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
            swing_time_R=t(p-1)-t_TOR_pre           % SWING TIME of previous step
            stride_time_R=t(p-1)-t_HSR_pre          % STRIDE TIME of previous step           
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
                stance_time_R=t(k-1)-t_HSR_pre          % STANCE TIME of step "in time"
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
            swing_time_L=t(P-1)-t_TOL_pre           % SWING TIME  of previous step
            stride_time_L=t(P-1)-t_HSL_pre          % STRIDE TIME of previous step            
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
                stance_time_L=t(K-1)-t_HSL_pre          % STANCE TIME of step "in time"
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
