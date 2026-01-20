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

Xtrue_ecef = nan(3, T);
for t = 1:T
    if ~isnan(Xloc(1,t))
        Xtrue_ecef(:,t) = P0 + M * Xloc(:,t);
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
plot3(Xtrue_ecef(2,:), Xtrue_ecef(1,:), Xtrue_ecef(3,:), 'LineWidth', 1.5); hold on;
plot3(Xhat_ecef(2,:), Xhat_ecef(1,:), Xhat_ecef(3,:), 'LineWidth', 1.5);
grid on; axis equal;

xlabel('E (m)');
ylabel('N (m)');
zlabel('D (m)');

legend('Vraie (Xloc)', 'Estimée');
title('Trajectoire 3D en repère global (N,E,D)');

view(3);

%% Simulation interférences et multitrajets (question 4)
sigmas = [0 1 2 5 10 20];
biases = [0 1 2 5 10 20 50];

sat_biased = 1;

rmse_noise = zeros(size(sigmas));
rmse_bias  = zeros(size(biases));

%% interférences
for kk = 1:numel(sigmas)
    sigma = sigmas(kk);

    PRN_used = PRN;
    for t = 1:T
        idx = find(~isnan(PRN_used(:,t)));
        for i = 1:numel(idx)
            s = idx(i);
            PRN_used(s,t) = PRN_used(s,t) + sigma * randn();
        end
    end

    Xhat_ecef = nan(3, T);
    bhat      = nan(1, T);
    maxIter = 10;
    X0 = [P0; 0];

    for t = 1:T
        idx = find(~isnan(PRN_used(:,t)));
        N   = numel(idx);
        if N < 4
            continue;
        end

        rho = PRN_used(idx, t);
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
            dX   = (H.'*H) \ (H.'*drho);
            X    = X + dX;
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

    ssum = 0; ccount = 0;
    for t = 1:T
        if ~isnan(err(t))
            ssum = ssum + err(t)^2;
            ccount = ccount + 1;
        end
    end
    mse_noise(kk) = ssum / ccount;
end

figure;
plot(sigmas, mse_noise, 'LineWidth', 1.5);
grid on;
xlabel('\sigma (m)'); ylabel('EQM (m^2)');
title('Robustesse aux interférences : EQM en fonction de \sigma');

%% multitrajets 
for kk = 1:numel(biases)
    B = biases(kk);

    PRN_used = PRN;
    for t = 1:T
        if ~isnan(PRN_used(sat_biased, t))
            PRN_used(sat_biased, t) = PRN_used(sat_biased, t) + B;
        end
    end

    Xhat_ecef = nan(3, T);
    bhat      = nan(1, T);
    maxIter   = 10;
    X0        = [P0; 0];

    for t = 1:T
        idx = find(~isnan(PRN_used(:,t)));
        N   = numel(idx);
        if N < 4
            continue;
        end

        rho = PRN_used(idx, t);
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
            dX   = (H.'*H) \ (H.'*drho);
            X    = X + dX;
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

    ssum = 0; ccount = 0;
    for t = 1:T
        if ~isnan(err(t))
            ssum = ssum + err(t)^2;
            ccount = ccount + 1;
        end
    end

    mse_bias(kk) = ssum / ccount;
end


figure;
plot(biases, mse_bias, 'LineWidth', 1.5);
grid on;
xlabel('Biais B (m)'); ylabel('EQM (m^2)');
title(['Robustesse aux multitrajets : EQM en fonction de B (sat ', num2str(sat_biased), ')']);
