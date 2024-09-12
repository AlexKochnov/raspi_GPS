
clc
clear variables
global x0 L lb ub sats sats_alm sats_eph

% sats_alm = readtable('alm_data.csv');
% sats_eph = readtable('eph_data.csv');
sats_alm = readtable('data/alm_6_.csv');
sats_eph = readtable('data/eph_6_2.csv');

% sats_alm = sats_alm(sats_alm.prMes > 0 & sats_alm.nav_score >0 & sats_alm.coord_score>0 &~isnan(sats_alm.X), :);
% sats_alm = sortrows(sats_alm, 'nav_score');
% sats_alm = sats_alm(1:8, :);
% sats_eph = sats_eph(sats_eph.prMes > 0 & sats_eph.nav_score >0 & sats_eph.coord_score>0 & ~isnan(sats_eph.X), :);


% sats = [table.X, table.Y, table.Z, table.prMes]
global C ECEF H Re
C = 299792458;
H = 140;
Re = 6363726; % вычислен для 140 метров тут https://rechneronline.de/earth-radius/, уже содержит 140 метро

sats_alm.prCorrected = sats_alm.prMes + sats_alm.af_dt * C
sats_eph.prCorrected = sats_eph.prMes + sats_eph.af_dt * C

min_func = @minimize_func;

lla2ecef = @MYlla2ecef;
ecef2lla = @MYecef2lla;

LLA = [55.690555555555555, 37.858333333333334, 140];

ECEF = lla2ecef(LLA);
x0 = [ECEF, 4e-3*C];
% x0 = [Re 0 0 0];
% contrains(x0)

% contrains(x0)

okno = 1000000;
lb = [ECEF - okno .* sign(ECEF), -1e-2]'; % dt
ub = [ECEF + okno .* sign(ECEF), +1e-2]';

lb = [ECEF - okno .* sign(ECEF), -1e+9]'; %cdt
ub = [ECEF + okno .* sign(ECEF), +1e+9]';

[lb(1:3), x0(1:3)', ub(1:3)]

% process('alm')
result = process('eph');
result'
sqrt(sum(x0(1:3).^2)) - Re
sqrt(sum(result(1:3).^2)) - Re

disp('satellites');
Rs = 26559.53861 * 10^3;
sqrt(sats.X.^2 + sats.Y.^2 + sats.Z.^2) - Rs
sqrt((sats.X-ECEF(1)).^2 + (sats.Y-ECEF(2)).^2 + (sats.Z-ECEF(3)).^2) 
sqrt((sats.X-ECEF(1)).^2 + (sats.Y-ECEF(2)).^2 + (sats.Z-ECEF(3)).^2)  - sats.prMes


[X,Y,Z] = sphere(500);
figure(1)
clf
scatter3(sats.X, sats.Y, sats.Z, 25, 'blue', 'filled');
hold on;
scatter3(result(1), result(2), result(3), 150, 'red', 'filled');
hold on;
scatter3(x0(1), x0(2), x0(3), 50, 'green', 'filled');
hold on;
surf(X * Re, Y * Re, Z * Re,'FaceAlpha',0.8, 'EdgeColor','none')
hold on;
surf(X * Rs, Y * Rs, Z * Rs,'FaceAlpha',0.2, 'EdgeColor','none')
hold off
grid on;
grid minor;
xlabel('X');
ylabel('Y');
zlabel('Z');


function result = process(new_mode)
    global x0 L lb ub sats sats_alm sats_eph

    if strcmp(new_mode, 'alm')
        sats = sats_alm;
        disp("      <strong>ALM</strong>")
    else
        sats = sats_eph;
        disp("      <strong>EPH</strong>")
    end
    % fprintf("<strong>METHOD:</strong> fminsearch\n")
    % result = fminsearch(@(params) norm(minimize_func(params, sats)), x0);
    % print_result(result, norm(minimize_func(result, sats)));

    % fprintf("\n<strong>METHOD:</strong> auto fmincon\n")
    % options = optimoptions('fmincon', 'Display', 'none');
    % result = fmincon(@(params) norm(minimize_func(params, sats)), x0, [], [], [], [], lb, ub, [], options);
    % print_result(result, norm(minimize_func(result, sats)));

    % fprintf("\n<strong>METHOD:</strong> SQP\n")
    % sqp_options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none');
    % [result, fval, exitflag, output] = fmincon(@(params) norm(minimize_func(params, sats)), x0, [], [], [], [], lb, ub, [], sqp_options);
    % print_result(result, fval);
    
    fprintf("\n<strong>METHOD:</strong> SQP с ограничением на высоту от земли H\n")
    sqp_options = optimoptions('fmincon', 'Algorithm', 'sqp');
    sqp_options = optimoptions('fmincon', 'Algorithm', 'interior-point');
    sqp_options.ConstraintTolerance = 1e-14;
    sqp_options.OptimalityTolerance = 1e-14;
    sqp_options.StepTolerance = 1e-14;
    sqp_options.FunctionTolerance = 1e-14;
    sqp_options.FiniteDifferenceType = 'central';
    sqp_options.Display = 'iter-detailed';
    sqp_options.MaxIterations = 50000;
    sqp_options.MaxFunctionEvaluations = 1e+6;
    [result, fval, exitflag, output] = fmincon(@(params) norm(minimize_func(params, sats)), x0*0, [], [], [], [], lb, ub, @contrains, sqp_options);
    print_result(result, fval);

    % opts = optimoptions('ga');
    % opts.ConstraintTolerance = 1e-10; % Точность выхода по ограничениями линейным и нелинейным
    % opts.PopulationSize = 50; % Размер популяции
    % opts.CrossoverFraction = 0.2; % Часть популяции, которая окажет влияние на следующее поколение (не считая элиты)
    % opts.EliteCount = 0.5 * opts.PopulationSize; % Количество элитных особей, которые выживут в следующем поколении
    % opts.FitnessLimit = -Inf; % Выход по минимуму минимизационной функции
    % opts.FunctionTolerance = 1e-10; % Точность выхода нахождения минимума
    % opts.MaxGenerations = 1000; % Выход по количеству итераций (количество сгенерированных поколений)
    % opts.MigrationDirection = 'both'; % Направление поиска двухстороннее
    % opts.MaxStallGenerations = 8;
    % opts.MaxTime = 60*30; % Можно задать максимальное время
    % opts.Display = 'iter';
    % opts.UseParallel = false;    
    % [result, fval, exitflag, output] = ga(@(params) norm(minimize_func(params, sats)), 4, [], [], [], [], [], [], @contrains, opts);
    % print_result(result, fval);
    % fprintf("расстоение от границы запрещенной зоны: %f\n", contrains(result))
    
    fprintf("\n<strong>METHOD:</strong> Levenberg-Marquardt \n")
    LM_options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'Display', 'none');
    [res, resnorm, residual, exitflag, output] = lsqnonlin(@(params) minimize_func(params, sats), x0, lb, ub, LM_options);
    print_result(res, norm(minimize_func(res, sats)));
    
    % fprintf("\n<strong>METHOD:</strong> GO - глобальная оптимизация \n")
    % go_options = optimoptions('fmincon', 'Algorithm', 'interior-point', 'Display', 'none');
    % problem = createOptimProblem('fmincon', 'objective', @(params) norm(minimize_func(params, sats)), 'x0', x0, 'lb', lb, 'ub', ub, 'options', go_options);
    % gs = GlobalSearch;
    % [result, fval, exitflag, output] = run(gs, problem);
    % print_result(result, fval);
end

function dists = minimize_func(params, sats)
    x = params(1);  
    y = params(2);
    z = params(3);
    dt = params(4); 
    % dists = sats.prCorrected - ( sqrt((sats.X - x).^2 + (sats.Y - y).^2 + (sats.Z - z).^2 ) + 299792458 * dt );
    dists = sats.prCorrected - ( sqrt((sats.X - x).^2 + (sats.Y - y).^2 + (sats.Z - z).^2 ) + dt );
    % dists = sum(dists.^2);
end

function print_result(result, fval)
    global C  ECEF Re ATT
    fprintf("true coords: (%f, %f, %f) m\n R= %f, R0 = %f, dR = %f \n", ECEF(1), ECEF(2), ECEF(3), sqrt(sum(ECEF.^2)), Re, abs(Re - sqrt(sum(ECEF.^2))))
    fprintf("computed coords: (%f, %f, %f) m, dt=%f s\n R= %f, d R = %f \n", ...
        result(1), result(2), result(3), result(4)/C, sqrt(sum(result(1:3).^2)), abs(Re - sqrt(sum(result(1:3).^2))))
    fprintf("calced error, m: %f\n" , (sqrt(sum((ECEF - result(1:3)).^2))))
    fprintf("function value: %f\n", fval)
    lla = ecef2lla(result(1:3));
    fprintf("lat: %f, lon: %f, alt: %f\n", lla(1), lla(2), lla(3));
end

function [c, ceq] = contrains(x)
    global Re ATT;

    % c = sqrt( x(1)^2 + x(2)^2 + x(3)^2 ) - Re;
    c = [];
    ceq= sqrt( x(1)^2 + x(2)^2 + x(3)^2 ) - Re;
end


function xyz = MYlla2ecef(lla)
    global Re;
    a = 6378137.0; % Полуось WGS-84 в метрах
    a = Re;
    f = 1 / 298.257223563; % Сжатие WGS-84
    e2 = 2 * f - f^2; % Квадрат эксцентриситета

    a =  6378137;
    b = 6356752.3142;

    e2 = 1 - b^2/a^2;

    lat = deg2rad(lla(:, 1));
    lon = deg2rad(lla(:, 2));
    alt = lla(:, 3);

    N = a ./ sqrt(1 - e2 * sin(lat).^2);

    x = (N + alt) .* cos(lat) .* cos(lon);
    y = (N + alt) .* cos(lat) .* sin(lon);
    z = (N * (1 - e2) + alt) .* sin(lat);

    xyz = [x, y, z];
end

function lla = MYecef2lla(xyz)
    x = xyz(:,1);
    y = xyz(:,2);
    z = xyz(:,3);
    
    
    % WGS84 ellipsoid constants:
    a = 6378137;
    es = (8.1819190842622e-2)^2;
    
    % calculations:
    b   = sqrt(a^2*(1-es));
    ep  = (a^2-b^2)/b^2;
    
    p   = sqrt(x.^2+y.^2);
    th  = atan2(a*z,b*p);
    lon = atan2(y,x);
    lat = atan2((z+ep^2.*b.*sin(th).^3),(p-es^2.*a.*cos(th).^3));
    N   = a./sqrt(1-es.*sin(lat).^2);
    alt = p./cos(lat)-N;
    
    % return lon in range [0,2*pi)
    %lon = mod(lon,2*pi); %atan2 should handle this fine
    
    % correct for numerical instability in altitude near exact poles:
    % (after this correction, error is about 2 millimeters, which is about
    % the same as the numerical precision of the overall function)
    
    k=abs(x)<1 & abs(y)<1;
    alt(k) = abs(z(k))-b;
    
    lla = zeros(size(xyz));
    lla(:,1) = lat*(180/pi);
    lla(:,2) = lon*(180/pi);
    lla(:,3) = alt;
end
