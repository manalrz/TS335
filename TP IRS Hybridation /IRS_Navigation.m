function [vAT_ins, psi_ins, pn_ins, pe_ins,  vn_ins, ve_ins] = IRS_Navigation ...
    (aAT_ins, aAT_ins_, HeadingRate_ins, HeadingRate_ins_, vAT_ins_, psi_ins_, pn_ins_, pe_ins_, dT)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function implement IRS navigation (dead-reckoning navigation).
% Inertial Reference System (IRS) mechanisation in Local Tangent Plane
% (LTP).
% 2D IRS simplified mechanisation - 1 gyro (z) + 1 accelero (x)
%
% Inputs
%   1) aAT_ins     Current Along track (AT) acceleration measurement [m/s²]
%   2) aAT_ins_     Previous Along track (AT) acceleration measurement [m/s²] - used for
%   linear interpolation
%   3) HeadingRate_ins     Current Heading rate measurement [rad/s]
%   4) HeadingRate_ins     Previous Heading rate measurement [rad/s] - used for
%   linear interpolation
%   5) vAT_ins_    Previous Along track (AT) velocity estimate [m/s]
%   6) psi_ins_    Previous heading estimate [rad]
%   7) pn_ins_     Previous position estimate in LTP - North axis [m]
%   8) pe_ins_     Previous position estimate in LTP - East axis [m]
%   9) dT          IMU sampling period [s]
%
% Outputs
%   1) vAT_ins     Current Along track (AT) velocity estimate [m/s]
%   2) psi_ins     Current heading estimate [rad]
%   3) pn_ins      Current position estimate in LTP - North axis [m]
%   4) pe_ins      Current position estimate in LTP - East axis [m]
%   5) pn_ins      Current velocity estimate in LTP - North axis [m/s]
%   6) pe_ins      Current velocity estimate in LTP - East axis [m/s]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright © ENAC, 2026
% ENAC : http://www.enac.fr/
% signav@recherche.enac.fr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 1) Heading integration (gyro z)
psi_ins = psi_ins_ + 0.5 * (HeadingRate_ins + HeadingRate_ins_) * dT;

% 2) Along-track velocity integration (acc x)
vAT_ins = vAT_ins_ + 0.5 * (aAT_ins + aAT_ins_) * dT;

% 3) Velocity components in LTP (North / East)
vn_ins = vAT_ins * cos(psi_ins);
ve_ins = vAT_ins * sin(psi_ins);

vn_ins_ = vAT_ins_ * cos(psi_ins_);
ve_ins_ = vAT_ins_ * sin(psi_ins_);

% 4) Position integration (trapézoidal)
pn_ins = pn_ins_ + 0.5 * (vn_ins + vn_ins_) * dT;
pe_ins = pe_ins_ + 0.5 * (ve_ins + ve_ins_) * dT;

end
