function [gyrX, gyrY, gyrZ, accX, accY, accZ, EulersGT, samplePeriod]= Data_reading(Dataset)
todeg = 180/pi;
gravity = 9.81;
if(Dataset ==1) %LOPSI test set
    data = load('Datasets/logfile_web_straight_LOPSI.mat');
    samp_s = 1;
    
    Gyr =(data.Gyr)*todeg;
    Acc =(data.Acc)/gravity;
    %     samp_e = length(data.Gyr);
    samp_e = 10000;
    EulersGT=data.Euler(samp_s:samp_e,:)*todeg;%[roll pitch yaw]
    
    samplePeriod = 1/100;
    gyrX = Gyr(samp_s:samp_e,1);%degree/s
    gyrY = Gyr(samp_s:samp_e,2);
    gyrZ = Gyr(samp_s:samp_e,3);
    accX = Acc(samp_s:samp_e,1);%g
    accY = Acc(samp_s:samp_e,2);
    accZ = Acc(samp_s:samp_e,3);
    
elseif(Dataset < 3 && Dataset > 2)
    start = 1; 
    if(Dataset == 2.1)
        filePath = 'Datasets/spiralStairs';
%         filePath_GT = 'Datasets/spiralStairs_GT.mat';
        stop = 12000;        
    elseif(Dataset == 2.2)
        filePath = 'Datasets/straightLine';
%         filePath_GT = 'Datasets/straightLine_GT.mat';
        stop = 7500;
    elseif(Dataset == 2.3)  
        filePath = 'Datasets/stairsAndCorridor';
%         filePath_GT = 'Datasets/stairsAndCorridor_GT.mat';
        stop = 10000;
    end
    samplePeriod = 1/256;
    xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate', 1/samplePeriod);
%     time = xIMUdata.CalInertialAndMagneticData.Time;
    gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
    gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
    gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
    accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
    accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
    accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
    clear('xIMUdata');
    gyrX = gyrX(start:stop,:);
    gyrY = gyrY(start:stop,:);
    gyrZ = gyrZ(start:stop,:);
    accX = accX(start:stop,:);
    accY = accY(start:stop,:);
    accZ = accZ(start:stop,:);
%     data = [gyrX, gyrY, gyrZ, accX, accY, accZ, magX, magY, magZ];
%     dataset = struct2array(load(filePath_GT));
    EulersGT = zeros(stop,3);%RPY
elseif (Dataset == 3)%Guo's FKF
    samplePeriod = 1/500;
    start = 1;
    %     stop = start+9999;
    data=load('Datasets/flat_motion2.txt');
    conv=diag([-1 1 -1]);
    Accelerometer=data(:,4:6)*conv;
     stop = length(Accelerometer);
    accX = Accelerometer(start:stop,1);
    accY = Accelerometer(start:stop,2);
    accZ = Accelerometer(start:stop,3);
    
    Gyroscope = data(:,7:9)*180/pi*conv;
    gyrX = Gyroscope(start:stop,1);
    gyrY = Gyroscope(start:stop,2);
    gyrZ = Gyroscope(start:stop,3);
    EulersGT=data(:,1:3)*conv;
    EulersGT(:,3) = EulersGT(:,3) + 180;
elseif(Dataset == 4) %BROAD
        data_in = load('Datasets/BROAD/input2.mat');
        data_out = load('Datasets/BROAD/output2.mat');    
    stop = 30000;
    gyrX =data_in.in_temp(1:stop,4)*180/pi;%deg/s
    gyrY =data_in.in_temp(1:stop,5)*180/pi;
    gyrZ =data_in.in_temp(1:stop,6)*180/pi;
    accX =data_in.in_temp(1:stop,1)/9.81;%g
    accY =data_in.in_temp(1:stop,2)/9.81;
    accZ =data_in.in_temp(1:stop,3)/9.81;
    
    EulersGT = quat2eul(data_out.out_temp(1:stop,1:4),'XYZ')*180/pi;
    samplePeriod = 1/286;
end