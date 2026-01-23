%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% XYZ = f_llh_2_xyz(llh)                                                    %
%                                                                           %
% Conversion des coordonnees (Latitude,Longitude,Hauteur) en coordonnées    %
% rectangulaires (X,Y,Z) ECEF dans le WGS-84.                               %
%                                                                           %
% Entrée : - LLH [rad, rad, m]                                              %
% Sortie : - XYZ [m, m, m]                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function XYZ = f_llh_2_xyz(llh)

a = 6378137.0;     % Demi grand axe de l'ellipsoide de reférence WGS-84 (m)
b = 6356752.3142;  % Demi petit axe de l'ellipsoide de reférence WGS-84 (m)
f = (a-b)/a;       % Aplatissement
e = sqrt(f*(2-f)); % Excentricité de l'ellipsoide WGS-84
N_lat = a/sqrt(1-e^2*sin(llh(1))^2);
XYZ(1) = (N_lat+llh(3)).*cos(llh(1)).*cos(llh(2));
XYZ(2) = (N_lat+llh(3)).*cos(llh(1)).*sin(llh(2));
XYZ(3) = (N_lat*(1-e^2)+llh(3)).*sin(llh(1));