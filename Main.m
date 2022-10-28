clear;
close all;
clc;
addpath('Quaternions'); %Quaternion library
addpath('ximu_matlab_library');
gravity = 9.81;
todeg = 180/pi;
torad = pi/180;

%% Data reading
Dataset = 1; 
our_sigma = 10^-4;

% Parameters for tracking algorithm
if(Dataset == 4)
    P =0.01*eye(4);                  
    Q = 0.001*eye(4);
    R = 0.025*eye(4);
else
    P =0.1*eye(4);
    Q = 0.01*eye(4);
    R = 0.025*eye(4);
end
%Reading data from datasets
[gyrX, gyrY, gyrZ, accX, accY, accZ, EulersGT, samplePeriod] = Data_reading(Dataset);
data  =[gyrX, gyrY, gyrZ, accX, accY, accZ, EulersGT];

%% Compute orientation
true_gyr = zeros(length(gyrX), 3);
quat = zeros(length(gyrX), 4); % Tracked orientation
quat_est = zeros(length(gyrX)+1, 4);% Predicted quaternion (orientation)
quat_est(1,:)=[1,0,0,0];

AHRSalgorithm = Tracking('SamplePeriod', samplePeriod, 'Kp', 0.5, 'KpInit', 1);
Predict = Prediction('SamplePeriod', samplePeriod);

%reading tracked orientation
if(Dataset >= 3 || Dataset < 2)
    if(Dataset == 3)
        data = load('Datasets/Qtracking_FKF_Guo.txt');
    elseif(Dataset == 1)
        data = load('Datasets/Qtracking_FKF_LOPSI.txt');
    elseif(Dataset == 4)
        data = load('Datasets/Qtracking_FKF_BROAD2.txt');
    end
    quat = data;
end

tic
for t = 1:length(gyrX) 
    %for dataset without tracked orientation
    if((Dataset ~= 3 && Dataset ~=1) && Dataset < 4)
        AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
        quat(t,:) = AHRSalgorithm.Quaternion; %qES
    end
    Kp = 0.5;
    
    % Prediction
    if(t < length(gyrX))
        if(t>2)
            Predict.proposedPred(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), true_gyr(t-1,:), true_gyr(t-2,:), quat(t,:), quat(t-1,:), quat(t-2,:), Kp, [accX(t) accY(t) accZ(t)], t, our_sigma);
        else
            Predict.firstPred(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]),quat(t,:), Kp, [accX(t) accY(t) accZ(t)]);% No error term
        end
        true_gyr(t,:) = Predict.true_w;
        quat_est(t+1,:) = Predict.qES;
    end
end

%Converting quaternion to Euler angles
Eulers_est= quatern2euler((quat_est(2:end-1,:)))*todeg;%ZYX, qES

Time_est = toc;
% tocf
if(Dataset > 2 && Dataset< 3)
    EulersGT = quatern2euler(quat(2:end,:))*todeg;%ZYX;
    EulersGT = [EulersGT(:,3), EulersGT(:,2), EulersGT(:,1)];
elseif(Dataset == 1)
    EulersGT = EulersGT(2:end,:);
end

RMSE_roll_estGT = RMSE (Eulers_est(2:end,3),EulersGT(2:end,1)); % x
RMSE_pitch_estGT = RMSE (Eulers_est(2:end,2),EulersGT(2:end,2)); % y
RMSE_yaw_estGT = RMSE (Eulers_est(2:end,1),EulersGT(2:end,3));% z
RMSE_overall_estGT = RMSE ([Eulers_est(2:end,3),Eulers_est(2:end,2),Eulers_est(2:end,1)],EulersGT(2:end,:));
RMSE_val = [RMSE_roll_estGT, RMSE_pitch_estGT,RMSE_yaw_estGT,RMSE_overall_estGT];

%% sub-plot Eulers
if(Dataset ==1)
    start = 1;
    stop = 1500;
elseif(Dataset ==3)
    start = 1;
    stop = 4000;
elseif(Dataset ==4)
    start = 2000;
    stop = 5000;
else
    start = 1;
    stop = length(EulersGT);
end
font_size = 28;
font_size_legend=26;
LineWidth = 3;
font_name = 'Times New Roman';
figure_roll=figure('Position', get(0, 'Screensize'), 'NumberTitle', 'off', 'Name', 'Eulers');
set(gca,'Fontsize',font_size,'FontName',font_name);
hold on;
ax(1) = subplot(3,1,1);
plot(start:1:stop, EulersGT(start:stop,1), 'k','LineWidth',LineWidth);
hold on;
ours =plot(start:1:stop, Eulers_est(start:stop,3), 'r','LineWidth',LineWidth);
hold off;
ylabel('R(Deg)','Fontsize',font_size, 'FontName','Times New Roman');
legend('GT', 'Proposed','Fontsize',font_size_legend,'Orientation','horizontal');

ax(2) = subplot(3,1,2);
plot(start:1:stop, EulersGT(start:stop,2), 'k','LineWidth',LineWidth);
hold on;
plot(start:1:stop, Eulers_est(start:stop,2), 'r','LineWidth',LineWidth);
hold off;
ylabel('P(Deg)','Fontsize',font_size, 'FontName','Times New Roman');
ax(3) = subplot(3,1,3);
plot((start:1:stop), EulersGT(start:stop,3), 'k','LineWidth',LineWidth);
hold on;
plot(start:1:stop, Eulers_est(start:stop,1), 'r','LineWidth',LineWidth);
hold off;
ylabel('Y(Deg)','Fontsize',font_size, 'FontName','Times New Roman');
xlabel('Samples','Fontsize',font_size, 'FontName','Times New Roman');
