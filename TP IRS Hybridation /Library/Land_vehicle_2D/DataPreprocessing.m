function DataPreprocessing(bTrajectory)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Preprocess recorded data: reference path, IMU path and ubx GPS path.
%
% Input:
%   bTrajectory : boolean for choosing the trajectory. If 1, sub-urban
%   environment, else harsh environment
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright © ENAC, 2016
% ENAC : http://www.enac.fr/
% signav@recherche.enac.fr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath(genpath('./ENAC_Car'));


%% Choose travel path
if nargin == 0 
    bTrajectory = 0; 
end

%% Reference trajectory - from trajectography equipment
% =========================
% Load reference data
% ---------------------------------------
if bTrajectory == 1
    load('novatelRef.mat'); % [time, lat[rad], long[rad], alt[m], ve, vn, vu, heading[°], pitch, roll]
else
    load('novatelRefTour.mat'); % [time, lat[rad], long[rad], alt[m], ve, vn, vu, heading[°], pitch, roll]
    novatelReftemp = [refData.Time, refData.posLLA, refData.velEnu, refData.att*180/pi];
    novatelRef = novatelReftemp(79095:100:107891,:);
end

t_Ref = novatelRef(:,1); % Reference time vector
llh_Ref = novatelRef(:,2:4); % Reference trajectoy - lat[rad] lon[rad) h[m]
nPts_Ref = length(t_Ref); % Number of samples of the reference trajectory
XYZ0 = f_llh_2_xyz(llh_Ref(1,:)); % Center of LTP - Initial position - in ECEF (X,Y,Z)
ned_Ref = zeros(nPts_Ref,3);  % Reference position in LTP  [m]
for i=1:nPts_Ref
    XYZ = f_llh_2_xyz(llh_Ref(i,:));
    ned_Ref(i,:) = f_xyz_2_NED(XYZ, XYZ0);
end;
heading_Ref =  novatelRef(:,8)*pi/180; % Reference heading [rad]
theta_Ref =  novatelRef(:,9)*pi/180; % Reference pitch [rad]
v_ned_Ref = [novatelRef(:,6), novatelRef(:,5), -novatelRef(:,7)]; % Reference NED velocity [m/s]

% Reference solution data file
% ---------------------------------------
save 'Reference.mat' t_Ref nPts_Ref llh_Ref ned_Ref v_ned_Ref heading_Ref theta_Ref XYZ0

%% IMU data - from Xsens MTi Inertial Measurement Unit (low-cost)
% =========================
% Load IMU data
% ---------------------------------------
if bTrajectory == 1
    load('ImuData.mat');
else
    load('ImuDataTour.mat');
    ImuData = imuData2;
end    

t_IMU =  ImuData(:,1); % IMU time vector
f_bi_p = [ImuData(:,3),-ImuData(:,2),ImuData(:,4)]; % Specific force in platform frame - from accelerometer - [m/s²]
w_bi_p = [ImuData(:,6),-ImuData(:,5),ImuData(:,7)]; % ImuData(:,5:7); % Mobile inertial angular rate wrt in platform frame - from gyrometer - [rad/s]
nPts_IMU = length(t_IMU); % Number of IMU measurements samples
Fs = 100; % IMU sampling frequency [Hz]
Ts = 1/Fs; % IMU sampling period [s]

% IMU data file (measurements in platform frame (p-frame))
% ---------------------------------------
save 'IMU.mat' t_IMU nPts_IMU Fs Ts f_bi_p w_bi_p

%% u-blox data (GPS)
% =========================
% Load GPS data
% ---------------------------------------
if bTrajectory == 1
    load('ubxData.mat');
    v_ned_GPS = [ubxData(:,9),ubxData(:,8),-ubxData(:,10)]; % GPS vehicle velocity in LTP wrt ECEF [m/s]
else
    load('ubxDataTour.mat');
    v_ned_GPS = [ubxData(:,6),ubxData(:,5),-ubxData(:,7)]; % GPS vehicle velocity in LTP wrt ECEF [m/s]
end    

t_GPS =  ubxData(:,1); % GPS time vector
xyz_GPS = ubxData(:,2:4); % GPS xyz vehicle position in ECEF frame - [m]
nPts_GPS = length(t_GPS); % Number of GPS measurements samples
ned_GPS = zeros(nPts_GPS, 3); % GPS position in LTP (NED) [m]
for i=1:nPts_GPS
    ned_GPS(i,:) = f_xyz_2_NED(xyz_GPS(i,:), XYZ0); % GPS position in LTP [m]
end;

% GPS data file
% ---------------------------------------
save 'GPS.mat' t_GPS nPts_GPS ned_GPS v_ned_GPS
