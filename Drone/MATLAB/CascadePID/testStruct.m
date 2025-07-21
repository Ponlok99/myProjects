clear
close all
clc
clear ekf10Step

pidAngle = struct('roll', [10,0,0,0], ...
                   'pitch', [10,0,0,0], ...
                   'yaw', [0,0,0,0]);

pidRate = struct('rollRate', [10,0,0,0], ...
                   'pitchRate', [10,0,0,0], ...
                   'yawRate', [0,0,0,0]);

doublePID = struct('pidAngle', pidAngle, ...
                   'pidRate', pidRate);

%% Define Angle Bus Object
angleBusInfo = Simulink.Bus;
angleBusInfo.Elements = Simulink.BusElement;
angleBusInfo.Elements(1).Name = 'roll';
angleBusInfo.Elements(1).DataType = 'double';
angleBusInfo.Elements(1).Dimensions = [1, 4];

angleBusInfo.Elements(2).Name = 'pitch';
angleBusInfo.Elements(2).DataType = 'double';
angleBusInfo.Elements(2).Dimensions = [1, 4];

angleBusInfo.Elements(3).Name = 'yaw';
angleBusInfo.Elements(3).DataType = 'double';
angleBusInfo.Elements(3).Dimensions = [1, 4];

%% Define Rate Bus Object
rateBusInfo = Simulink.Bus;
rateBusInfo.Elements = Simulink.BusElement;
rateBusInfo.Elements(1).Name = 'rollRate';
rateBusInfo.Elements(1).DataType = 'double';
rateBusInfo.Elements(1).Dimensions = [1, 4];

rateBusInfo.Elements(2).Name = 'pitchRate';
rateBusInfo.Elements(2).DataType = 'double';
rateBusInfo.Elements(2).Dimensions = [1, 4];

rateBusInfo.Elements(3).Name = 'yawRate';
rateBusInfo.Elements(3).DataType = 'double';
rateBusInfo.Elements(3).Dimensions = [1, 4];

%% PID combined Bus Info
combinedBusInfo = Simulink.Bus;
combinedBusInfo.Elements = Simulink.BusElement;
combinedBusInfo.Elements(1).Name = 'angles';
combinedBusInfo.Elements(1).DataType = 'Bus: angleBusInfo';
combinedBusInfo.Elements(1).Dimensions = 1;

combinedBusInfo.Elements(2).Name = 'rates';
combinedBusInfo.Elements(2).DataType = 'Bus: rateBusInfo';
combinedBusInfo.Elements(2).Dimensions = 1;

% %% imu bus Info
% imuInfo = Simulink.Bus;
% 
% imuInfo.Elements = Simulink.BusElement;
% imuInfo.Elements(1).Name = 'accel';
% imuInfo.Elements(1).DataType = 'double';
% imuInfo.Elements(1).Dimensions = [1, 3];
% 
% imuInfo.Elements(2).Name = 'gyro';
% imuInfo.Elements(2).DataType = 'double';
% imuInfo.Elements(2).Dimensions = [1, 3];
% 
% %% system bus info
% sysInfo = Simulink.Bus;
% 
% sysInfo.Elements = Simulink.BusElement;
% sysInfo.Elements(1).Name = 'dt';
% sysInfo.Elements(1).DataType = 'double';
% sysInfo.Elements(1).Dimensions = 1;
% 
% sysInfo.Elements(2).Name = 'Q';
% sysInfo.Elements(2).DataType = 'double';
% sysInfo.Elements(2).Dimensions = [6, 6];
% 
% sysInfo.Elements(3).Name = 'R_acc';
% sysInfo.Elements(3).DataType = 'double';
% sysInfo.Elements(3).Dimensions = [3, 3];
% 
% sysInfo.Elements(4).Name = 'R_mag';
% sysInfo.Elements(4).DataType = 'double';
% sysInfo.Elements(4).Dimensions = [3, 3];
% 
% sysInfo.Elements(5).Name = 'R_baro';
% sysInfo.Elements(5).DataType = 'double';
% sysInfo.Elements(5).Dimensions = 1;

%% Assign to base workspace
assignin('base', 'pidAngleBus', angleBusInfo);
assignin('base', 'pidRateBus', rateBusInfo);
assignin('base', 'doublePIDInfo', combinedBusInfo);
% assignin('base', 'imuInfo', imuInfo);
% assignin('base', 'sysInfo', sysInfo);

% load testahrsfilter2.mat
% 
% numSamples = length(tvec);
% imuFs = 1000;
% 
% gyroahr = timeseries(rawGyro, tvec);
% accahr = timeseries(rawAccel, tvec);
% magahr = timeseries(rawMag, tvec);
% setAngles = timeseries(zeros(numSamples,3), tvec);
