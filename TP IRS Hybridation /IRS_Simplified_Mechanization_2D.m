%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inertial Reference System (IRS) mechanisation in Local Tangent Plane
% (LTP).
% Simplified 2D mechanization for land vehicle.
%
% The script requires as input:
%   3D accelerometer measurements
%   3D gyroscope measurements
%   Initial attitude
%   Initial position
%   Initial velocity
%   Reference trajectory
%
% Dependencies:
%   DataPreprocessing.m
%   IRS_Navigation.m %------- Function to complete -------%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright © ENAC, 2026
% ENAC : http://www.enac.fr/
% signav@recherche.enac.fr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; 
close all; clc; format long
addpath(genpath('Library'));
iFigure=1;

%% Download data
% =========================
DataPreprocessing(0);
load ('Reference.mat');
load ('IMU.mat');

%% IMU raw measurements analysis
% =========================
% Reference path in LTP
% ---------------------------------------
close(figure(iFigure)); figure(iFigure);
plot(ned_Ref(:,2), ned_Ref(:,1), '.r', 'MarkerSize',24);
xlabel('East axis [m]'); ylabel('North axis [m]'); 
grid on;  title('2D reference path in LTP (ned-coordinate frame)');
iFigure=iFigure+1;
% Specific force along platform x-axis (p-frame)
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); 
plot(t_IMU-t_IMU(1), f_bi_p(:,1), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('Specific force [m/s²]');
grid on; title('Specific force measurement, f_b_/_i ^p, along platform x-axis');
iFigure=iFigure+1;
% Angular rate along platform z-axis (p-frame)
% ---------------------------------------
close(figure(iFigure)); figure(iFigure);
plot(t_IMU-t_IMU(1), w_bi_p(:,3), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('Angular velocity [rad/s]');
grid on; title('Gyrometer measurement, \omega_b_/_i ^p, along platform z-axis');
iFigure=iFigure+1;


%% IMU data in body frame
% =========================
R_platform2body = [1 0 0; 
                   0 -1 0; 
                   0 0 -1];
f_bi_b = f_bi_p*R_platform2body;
w_bi_b = w_bi_p*R_platform2body;

% Correct the heading rate measurement from initial gyro bias
% ---------------------------------------
HeadingRate_ins = w_bi_b(:,3); % Heading rate 
bgyro = 0; % The bias is estimated during the first 2s of data collection
HeadingRate_ins = HeadingRate_ins - bgyro; % rad/s
% Correct along track acceleration measurement from initial accelerometer bias
% ---------------------------------------
aAT_ins = f_bi_b(:,1); % Along track (AT) acceleration
bacc = 0; % The bias is estimated during the first 2s of data collection
aAT_ins = f_bi_b(:,1) - bacc; % m/s²


%% Initialize output variables
% =========================
psi_ins = zeros(nPts_IMU,1); % IRS heading estimate [rad]
vAT_ins = zeros(nPts_IMU,1); % Along track (AT) velocity estimate [m/s]
vn_ins = zeros(nPts_IMU,1); % IRS North velocity estimate in LTP [m/s]
ve_ins = zeros(nPts_IMU,1); % IRS East velocity estimate in LTP [m/s]
pn_ins = zeros(nPts_IMU,1); % IRS position estimate in LTP - North axis [m]
pe_ins = zeros(nPts_IMU,1); % IRS position estimate in LTP - East axis [m]


%% IRS implementation
% =========================

h = waitbar(0,'IRS navigation ...');
j=1;
for i=1:nPts_IMU
    
    if (mod(i,Fs) == 0) 
        waitbar(i / nPts_IMU);
        j=j+1;
    end
    
    % Simplified IRS 2D mechanisation
    % ============================

    if (i == 1) % Initialization
        dT = Ts;
        psi_ins(i) = heading_Ref(1);
        vAT_ins(i) = 0;
        vn_ins(i) = 0;
        ve_ins(i) = 0;
        pn_ins(i) = 0;
        pe_ins(i) = 0;

    else % IRS navigation
    
        [vAT_ins(i), psi_ins(i), pn_ins(i), pe_ins(i), vn_ins(i), ve_ins(i)] = ...
            IRS_Navigation( ...
                aAT_ins(i), aAT_ins(i-1), ...
                HeadingRate_ins(i), HeadingRate_ins(i-1), ...
                vAT_ins(i-1), psi_ins(i-1), ...
                pn_ins(i-1), pe_ins(i-1), ...
                dT );
    
    end
    
end
close(h);

%% Display Results
% =========================
t = [0:1/Fs:(nPts_IMU-1)/Fs]';
t_corr = [-(nPts_IMU-1)/Fs:1/Fs:(nPts_IMU-1)/Fs]';

% AT acceleration
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); 
subplot(2,1,1), plot(t, aAT_ins, 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[m/s²]');
grid on; title('Along Track acceleration');
subplot(2,1,2),plot(t_corr, xcorr(aAT_ins), 'b', 'LineWidth',2);
grid on; title('Autocorrelation function of the Along Track acceleration');
iFigure=iFigure+1;


% Heading rate
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); 
subplot(2,1,1),plot(t, HeadingRate_ins, 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[rad/s]');
grid on; title('Heading rate');
subplot(2,1,2),plot(t_corr, xcorr(HeadingRate_ins), 'b', 'LineWidth',2);
grid on; title('Autocorrelation function of the Heading rate');
iFigure=iFigure+1;

% 2D position in LTP
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); hold on; grid on; title('2D Position in LTP - NED frame');
plot(ned_Ref(:,2), ned_Ref(:,1), '.r', 'MarkerSize',24);
plot(pe_ins, pn_ins, '.-b', 'LineWidth',2, 'MarkerSize',12);
xlabel('East axis [m]'); ylabel('North axis [m]');
legend('REF','Simplified IRS');
iFigure=iFigure+1;

% Position error
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); 
subplot(2,1,1), plot(t_Ref-t_Ref(1),ned_Ref(:,1)-pn_ins(1:Fs:end), 'g', 'LineWidth',2);
grid on,
title('IRS position error (Reference position minus IRS position)');
xlabel('Time [s]'); ylabel('North position error [m]');
subplot(2,1,2), plot(t_Ref-t_Ref(1),ned_Ref(:,2)-pe_ins(1:Fs:end), 'k', 'LineWidth',2);
grid on,
xlabel('Time [s]'); ylabel('East position error [m]');
iFigure=iFigure+1;

% North-East velocity in LTP
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); 
subplot(2,1,1),plot(t_Ref-t_Ref(1), v_ned_Ref(:,1), 'r', 'LineWidth',2);hold on; grid on; 
plot(t_Ref-t_Ref(1), vn_ins(1:Fs:end), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('North velocity [m/s]'); legend('REF','Simplified IRS');
title('Vehicle velocity in LTP');
subplot(2,1,2),plot(t_Ref-t_Ref(1), v_ned_Ref(:,2), 'r', 'LineWidth',2);hold on; grid on;
plot(t_Ref-t_Ref(1), ve_ins(1:Fs:end), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('East velocity [m/s]'); 
iFigure=iFigure+1;

% Vehicle heading
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); 
subplot(2,1,1),hold on; grid on; title('Vehicle Heading (shifted phase angle)');
plot(t_Ref-t_Ref(1), 180/pi*unwrap(heading_Ref), 'r', 'LineWidth',2);
plot(t_Ref-t_Ref(1), 180/pi*unwrap(psi_ins(1:Fs:end)), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[deg]'); legend('REF','Simplified IRS');
subplot(2,1,2),hold on; grid on; title('Heading error: REF - IRS');
plot(t_Ref-t_Ref(1), 180/pi*unwrap(heading_Ref) - 180/pi*unwrap(psi_ins(1:Fs:end)), 'k', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[deg]');
iFigure=iFigure+1;
