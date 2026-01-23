%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calcul de g selon le modèle WGS-84                                        %
%                                                                           %
% Entrées:                                                                  %
%   - latitude L                                                            %
%   - Hauteur H                                                             %
%                                                                           %
% D. Kubrak - 21/08/2008                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function g = f_g_WGS(L,H)

a = 6378137;                    % rayon de la Terre à l'Equateur
b = 6356752.3142;               % distance entre le centre de la Terre et le pole
e = sqrt(6.694379991013e-3);    % excentricité
ge = 9.7803267714;              % ge intensité de la pesanteur à altitude nulle et à l'équateur
gp = 9.8321863665;              %gp intensité de la pesanteur à altitude nulle et aux poles

k = ((b*gp)/(a*ge))-1;
g0 = ge*(1+k*sin(L).^2)./sqrt(1-e^2*sin(L).^2);
g = g0.*((1./(a+H)).*a).^2;

