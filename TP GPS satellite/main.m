clear; close all; clc;

%% Chargement des données
load donnees_GPS_TP.mat
load trajectoire_TP.mat
[~, T] = size(PRN);

%% Coordonnées point de ref P0 (question 2)
alt = 0;
lat = deg2rad(44 + 48/60);
lon = - (deg2rad(0 + 35/60));

llh = [lat, lon, alt];
P0 = llh2xyz(llh);
P0 = P0(:);

%% Trajectoire véhicule (question 3)
M = ecef(lat, lon);

Xhat_ecef = nan(3, T);
bhat      = nan(1, T);

maxIter = 10;
tolDX   = 1e-3;

X0 = [P0; 0]; 

for t = 1:T

    idx = find(~isnan(PRN(:,t)));
    N   = numel(idx);

    if N < 4
        continue;
    end

    rho = PRN(idx, t);
    sat = zeros(3, N);

    for i = 1:N
        s = idx(i);
        sat(:,i) = [XYZsat(s,1,t); XYZsat(s,2,t); XYZsat(s,3,t)];
    end

    if t > 1 && ~isnan(bhat(t-1))
        X0 = [Xhat_ecef(:,t-1); bhat(t-1)];
    end

    X = X0;

    for k = 1:maxIter
        x = X(1:3);
        b = X(4);

        rhat = zeros(N,1);
        H    = zeros(N,4);

        for i = 1:N
            d = x - sat(:,i);
            r = norm(d);

            rhat(i)  = r + b;
            H(i,1:3) = (d / r).';
            H(i,4)   = 1;
        end

        drho = rho - rhat;

        A = H.'*H;

        dX = A \ (H.'*drho);
        X  = X + dX;

    end


    Xhat_ecef(:,t) = X(1:3);
    bhat(t)        = X(4);
    X0             = X;
end

Xhat_loc = nan(3, T);

for t = 1:T
    if ~isnan(Xhat_ecef(1,t))
        Xhat_loc(:,t) = M' * (Xhat_ecef(:,t) - P0);
    end
end

err = nan(1, T);

for t = 1:T
    if ~isnan(Xhat_loc(1,t))
        d = Xhat_loc(:,t) - Xloc(:,t);
        err(t) = sqrt(d(1)^2 + d(2)^2 + d(3)^2);
    end
end

s = 0; c = 0;
for t = 1:T
    if ~isnan(err(t))
        s = s + err(t)^2;
        c = c + 1;
    end
end

rmse = sqrt(s / c);
disp(['RMSE local (m) = ', num2str(rmse)]);

figure;
plot(Xloc(2,:), Xloc(1,:), 'LineWidth', 1.5); hold on;
plot(Xhat_loc(2,:), Xhat_loc(1,:), 'LineWidth', 1.5);
grid on; axis equal;
xlabel('E (m)'); ylabel('N (m)');
legend('Vraie (Xloc)', 'Estimée');
title('Trajectoire en repère local (N,E)');

figure;
plot(err, 'LineWidth', 1.5);
grid on;
xlabel('t (s)'); ylabel('||erreur|| (m)');
title(['Erreur position (RMSE = ', num2str(rmse,'%.2f'), ' m)']);