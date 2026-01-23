%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IRS/GPS loose coupling in Local Tangent Plane (LTP).
% Simplified 2D IRS mechanization for land vehicle.
%
% The script requires as input:
%   3D accelerometer measurements
%   3D gyroscope measurements
%   Initial attitude
%   Initial position
%   Initial velocity
%   GPS position estimates
%   GPS velocity estimate
%   Reference trajectory
%
% Dependencies:
%   DataPreprocessing.m
%   IRS_Navigation.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright © ENAC, 2026
% ENAC : http://www.enac.fr/
% signav@recherche.enac.fr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all; clc; format long
addpath(genpath('Library'));
iFigure = 1;


%% Download data
% =========================
% Choice of trajectory
bTrajectory = 0;
DataPreprocessing(bTrajectory);
load ('Reference.mat');
load ('IMU.mat');
load ('GPS.mat');


%% GPS navigation solution analysis
% =========================
% GPS trajectory in LTP
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); hold on; grid on; title('2D Position in LTP - NED frame');
plot(ned_Ref(:,2), ned_Ref(:,1), '.r', 'MarkerSize',24);
plot(ned_GPS(:,2), ned_GPS(:,1), '.g', 'MarkerSize',18);
xlabel('East axis [m]'); ylabel('North axis [m]');
legend('Reference','GPS');
iFigure=iFigure+1;
% GPS position error
% ---------------------------------------
close(figure(iFigure)); figure(iFigure);
subplot(2,1,1), plot(ned_Ref(:,1)-ned_GPS(:,1), 'k', 'LineWidth',2);
grid on,
title('GPS position error: REF - GPS');
xlabel('Time [s]'); ylabel('North position error [m]');
subplot(2,1,2), plot(ned_Ref(:,2)-ned_GPS(:,2), 'k', 'LineWidth',2);
grid on,
xlabel('Time [s]'); ylabel('East position error [m]');
iFigure=iFigure+1;
% GPS velocity error
% ---------------------------------------
close(figure(iFigure)); figure(iFigure);
subplot(2,1,1), plot(v_ned_Ref(:,1)-v_ned_GPS(:,1), 'k', 'LineWidth',2);
grid on,
title('GPS velocity error: REF - GPS');
xlabel('Time [s]'); ylabel('North velocity error [m/s]');
subplot(2,1,2), plot(v_ned_Ref(:,2)-v_ned_GPS(:,2), 'k', 'LineWidth',2);
grid on,
xlabel('Time [s]'); ylabel('East velocity error [m]');
iFigure=iFigure+1;

% return

%% IMU data in body frame
% =========================
R_platform2body = [1 0 0; 
                   0 -1 0; 
                   0 0 -1];
f_bi_b = f_bi_p*R_platform2body;
w_bi_b = w_bi_p*R_platform2body;

% Correct heading rate from initial gyro bias
% ---------------------------------------
HeadingRate_ins = w_bi_b(:,3); % Heading rate 
bgyro	= mean(HeadingRate_ins(1:2*Fs)); % The bias is estimated during the first 2s of data collection
HeadingRate_ins   = HeadingRate_ins - bgyro; % rad/s
% Correct along track acceleration from initial accelerometer bias
% ---------------------------------------
aAT_ins = f_bi_b(:,1); % Along track (AT) acceleration
bacc	= mean(aAT_ins(1:2*Fs)); % The bias is estimated during the first 2s of data collection
aAT_ins   = f_bi_b(:,1) - bacc; % m/s²


%% Initialize Kalman filter model
% =========================
% State progation model
StateVectorLenght = 6;
% Initialize error state vector - delta_X_IRS
X0 =  zeros(StateVectorLenght, 1);
% Initialize covariance matrix - cov(X0)
S0 = diag([5 5 5 1 0.1 1].^2);
% Measurement model
MeasVectorLenght = 2;
meas = zeros(MeasVectorLenght,1); % Initialize measurement vector
% Initialize Kalman filter internal variables
X_hat = X0; % X0|0
S = S0; % cov(X0|0)
% Initialize IMU sensor error corrections
gyro_bias = 0; % Gyromater bias correction
accelero_bias = 0; % Accelerometer bias correction


%% Initialize output variables
% =========================
% Inertial navigation solution
% ---------------------------------------
vAT_ins = zeros(nPts_IMU,1); % Along track (AT) velocity estimate [m/s]
psi_ins = zeros(nPts_IMU,1); % IRS heading estimate  [rad]
pn_ins = zeros(nPts_IMU,1); % IRS position estimate in LTP - North axis [m]
pe_ins = zeros(nPts_IMU,1); % IRS position estimate in LTP - East axis [m]
vn_ins = zeros(nPts_IMU,1); % IRS North velocity estimate in LTP [m/s]
ve_ins = zeros(nPts_IMU,1); % IRS East velocity estimate in LTP [m/s]
% Hybridized navigation solution
% ---------------------------------------
X_kf = zeros(StateVectorLenght,nPts_IMU);
cov_kf = zeros(StateVectorLenght,nPts_IMU);
Innov_kf = zeros(MeasVectorLenght,nPts_IMU);
cov_innov = zeros(MeasVectorLenght,nPts_IMU);
psi_GPS_IRS = zeros(nPts_IMU-1,1); % GPS/IRS heading estimate
pn_GPS_IRS = zeros(nPts_IMU-1,1); % GPS/IRS position estimate in LTP - North axis
pe_GPS_IRS = zeros(nPts_IMU-1,1); % GPS/IRS position estimate in LTP - East axis
vn_GPS_IRS = zeros(nPts_IMU-1,1); % GPS/IRS North velocity estimate in LTP
ve_GPS_IRS = zeros(nPts_IMU-1,1); % GPS/IRS East velocity estimate in LTP
vAT_GPS_IRS = zeros(nPts_IMU-1,1); % GPS/IRS East velocity estimate in LTP


%% Configure GPS/IRS coupling
% =========================
% Implement GPS signals masking: loss of GPS information during
% MaskingInterval seconds
% ---------------------------------------
Min_MaskingInterval = 100; % Start of masking interval
MaskingInterval = 0; % Masking interval must be inferior to 180 - 0, by default
Masking = zeros (1,nPts_GPS); % Initialize masking vector - 0 = no signals masking, else 1
MaskingInterval = Min_MaskingInterval:Min_MaskingInterval+MaskingInterval-1;
Masking(MaskingInterval) = 1;% Set masking


h = waitbar(0,'GPS/IRS hybridization ...');

iGPS = 1;
iKF = 0;
for i=1:nPts_IMU
    
    ins_corr = zeros(StateVectorLenght,1); % Initialize IRS correction vector
    
    % Simplified IRS 2D mechanisation
    % ============================
    if (i == 1)  % Initialize IRS platform
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
    
    % GPS/IRS coupling
    % ============================
    
    % Continuous-time state transition model
    % ---------------------------------------------------
    % State transition matrix - continuous time
    Ft = [ ...
    0  0  cos(psi_ins(i))             0  -vAT_ins(i)*sin(psi_ins(i))   0;
    0  0  sin(psi_ins(i))             0   vAT_ins(i)*cos(psi_ins(i))   0;
    0  0  0                           1   0                            0;
    0  0  0                          -1/5 0                            0;
    0  0  0                           0   0                            1;
    0  0  0                           0   0                            -1/5
    ];
    
    % Process noise covariance matrix - continuous time
    Qt = diag([0 0 0.1 0.05 0.01 0.001].^2);
    
    % Discrete-time state transition model
    % -------------------------------------------------
    Fk = eye(6) + dT*Ft + Ft*Ft*dT^2/2;
    Qk = Qt*dT + (Ft*Qt+Qt*Ft')*dT^2/2;
    
    % Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
    % ----------------------------------------------------------
    X_hat = Fk * X_hat; % X_hat = X_k|k-1
    S = Fk*S*Fk' + Qk; % S = cov(X_k|k-1)
    
    innov = zeros(MeasVectorLenght,1); % Initialize innovation vector
    V = zeros(MeasVectorLenght,MeasVectorLenght); % Initialise Innovation covariance
    
    if (mod(i,Fs) == 0)  % Test GPS data availability
        waitbar(i / nPts_IMU);
        if (Masking(iGPS) == 0) % Test GPS signal masking
            % Compute measurement model
            % ---------------------------------------------------------
            obs_GPS = [ned_GPS(iGPS,1); ned_GPS(iGPS,2)]; % Observation vector - GPS
            obs_pred = [pn_ins(i); pe_ins(i)]; % Observation vector prediction - IRS
            % Measurement vector
            meas = obs_GPS - obs_pred;  % Innovation mesurée = GPS - IRS
            
            % Measurement matrix
            H = [1 0 0 0 0 0; 0 1 0 0 0 0];
            
            % Measurement noise covariance matrix
            R = diag([10 10].^2);
            
            % Compute Kalman filter gain
            % ---------------------------------------------------------
            innov = meas - H*X_hat; % Innovation vector
            V = H*S*H' + R; % cov(innov)
            K = S*H'*inv(V); % Kalman filter gain
            
            % Correct state prediction (state estimation)
            % ---------------------------------------------------------
            X_hat = X_hat + K*innov; % X_hat = X_k|k
            S = S - K*H*S; % S = cov(X_k|k)
            
            % Save innovation vector
            % ---------------------------------------------------------
            iKF = iKF + 1;
            Innov_kf(:,iKF) = innov; % Innovation vector
            cov_innov(:,iKF) = diag(V); % cov(Innov_kf)
            
        end %end((Masking(iGPS) == 0))
        iGPS = iGPS + 1;
    end %end(if (mod(i,100) == 0) && (Masking(iGPS) == 0))
    
    % Save KF output
    % ----------------
    X_kf(:,i) = X_hat; % Inertial errors estimates
    cov_kf(:,i) = diag(S); % Estimate error covariance
    
    % GPS/IRS navigation solution
    pn_GPS_IRS(i)  = pn_ins(i) + X_hat(1);  % North position corrected
    pe_GPS_IRS(i)  = pe_ins(i) + X_hat(2);  % East position corrected
    psi_GPS_IRS(i) = psi_ins(i) + X_hat(3); % Heading corrected
    vAT_GPS_IRS(i) = vAT_ins(i) + X_hat(4); % Along track velocity corrected
    vn_GPS_IRS(i)  = vn_ins(i) + X_hat(5);  % North velocity corrected
    ve_GPS_IRS(i)  = ve_ins(i) + X_hat(6);  % East velocity corrected
    
end % end(for i=1:nPts_IMU)

close(h);

%% Display results
% =========================

% 2D position in LTP
% ---------------------------------------
close(figure(iFigure)); figure(iFigure); hold on; grid on; title('2D Position in LTP - NED frame');
plot(ned_Ref(:,2), ned_Ref(:,1), '.r', 'MarkerSize',24);
plot(ned_GPS(:,2), ned_GPS(:,1), '.g', 'MarkerSize',18);
plot(pe_GPS_IRS, pn_GPS_IRS, '.-b', 'LineWidth',2, 'MarkerSize',12);
xlabel('East axis [m]'); ylabel('North axis [m]');
legend('REF','GPS', 'GPS/IRS');
iFigure=iFigure+1;

% Position error
% ---------------------------------------
close(figure(iFigure)); figure(iFigure);
subplot(2,1,1), plot(ned_Ref(:,1)-pn_GPS_IRS(1:100:end), 'r', 'LineWidth',2);
grid on,
title('Hybridized position error: REF - GPS/IRS');
xlabel('Time [s]'); ylabel('North position error [m]');
subplot(2,1,2), plot(ned_Ref(:,2)-pe_GPS_IRS(1:100:end), 'k', 'LineWidth',2);
grid on,
xlabel('Time [s]'); ylabel('East position error [m]');
iFigure=iFigure+1;


% North-East velocity in LTP
% ---------------------------------------
close(figure(iFigure)); figure(iFigure);
subplot(2,1,1),plot(v_ned_Ref(:,1), 'r', 'LineWidth',2);hold on; grid on;
plot(vn_GPS_IRS(1:Fs:end), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('North velocity [m/s]'); legend('REF','GPS/IRS');
title('Vehicle velocity in LTP');
subplot(2,1,2),plot(v_ned_Ref(:,2), 'r', 'LineWidth',2);hold on; grid on;
plot(ve_GPS_IRS(1:Fs:end), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('East velocity [m/s]');
iFigure=iFigure+1;

% Vehicle heading
% ---------------------------------------
close(figure(iFigure)); figure(iFigure);
subplot(2,1,1), hold on; grid on; title('Vehicle Heading (shifted phase angle)');
plot(180/pi*unwrap(heading_Ref), 'r', 'LineWidth',2);
plot(180/pi*unwrap(psi_GPS_IRS(1:Fs:end)), 'b', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[deg]'); legend('REF','GPS/IRS');
subplot(2,1,2), hold on; grid on; title('Heading error: REF - GPS/IRS');
plot(180/pi*unwrap(heading_Ref)-180/pi*unwrap(psi_GPS_IRS(1:Fs:end)), 'k', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[deg]');
iFigure=iFigure+1;

% Kalman filter validation
% ---------------------------------------
% Estimation error
close(figure(iFigure)); figure(iFigure);
subplot(2,1,1), plot((ned_Ref(:,1) - pn_ins(1:Fs:end))'-X_kf(1,1:Fs:end), 'k', 'LineWidth',2), hold on, grid on,
plot(3*sqrt(cov_kf(1,1:Fs:end)), 'r', 'LineWidth',2);
plot(-3*sqrt(cov_kf(1,1:Fs:end)), 'r', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[m]'); legend('Estimation error','3\sigma envelop');
title('Estimation error of IRS position error (north axis)');
subplot(2,1,2), hist((ned_Ref(:,1) - pn_ins(1:Fs:end))'-X_kf(1,1:Fs:end), 100), grid on,
title('Estimation error histogram'),
iFigure=iFigure+1;

close(figure(iFigure)); figure(iFigure);
subplot(2,1,1), plot((ned_Ref(:,2) - pe_ins(1:Fs:end))'-X_kf(2,1:Fs:end), 'b', 'LineWidth',2), hold on, grid on,
plot(3*sqrt(cov_kf(2,1:Fs:end)), 'r', 'LineWidth',2);
plot(-3*sqrt(cov_kf(2,1:Fs:end)), 'r', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[m]'); legend('Estimation error','3\sigma envelop');
title('Estimation error of IRS position error (East axis)');
subplot(2,1,2), hist((ned_Ref(:,2) - pe_ins(1:Fs:end))'-X_kf(2,1:Fs:end), 50), grid on
title('Estimation error histogram'),
iFigure=iFigure+1;

% Innovation
close(figure(iFigure)); figure(iFigure);
subplot(2,1,1), plot(Innov_kf(1,1:iKF), 'k', 'LineWidth',2), hold on, grid on,
plot(3*sqrt(cov_innov(1,1:iKF)), 'r', 'LineWidth',2);
plot(-3*sqrt(cov_innov(1,1:iKF)), 'r', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[m]'); legend('Innovation','3\sigma envelop');
title('Innovation (North axis measurement)');
subplot(2,1,2), hist(Innov_kf(1,1:iKF), 100), grid on,
title('Innovation histogram'),


