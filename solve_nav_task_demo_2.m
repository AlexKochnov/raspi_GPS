
clc
clear variables
global x0 L lb ub sats sats_alm sats_eph

sats_alm = readtable('alm_data.csv');
sats_eph = readtable('eph_data.csv');

sats_alm = sats_alm(sats_alm.prMes > 0 & sats_alm.nav_score >0 & sats_alm.coord_score>0 &~isnan(sats_alm.X), :);
sats_alm = sortrows(sats_alm, 'nav_score');
sats_alm = sats_alm(1:8, :)
sats_eph = sats_eph(sats_eph.prMes > 0 & sats_eph.nav_score >0 & sats_eph.coord_score>0 & ~isnan(sats_eph.X), :)



% sats = [table.X, table.Y, table.Z, table.prMes]
global C ECEF H Re
C = 299792458;
H = 1000;
Re = 6378136;

% lla2ecef = @MYlla2ecef;
% ecef2lla = @MYecef2lla;

LLA = [55.569861111111116, 38.805027777777774, 140];

ECEF = MYlla2ecef(LLA);


% x0 = [0, 0, 0, 0];

% x0 = [2.8e6, 2.26e6, 5.23e6, 0];
% x0 = x0 / norm(x0) * Re;

x0 = [MYlla2ecef([55.57, 38.8, 100]), 4e-3*C];

% x0 = [ECEF, 4e-3*C];

contrains(x0)
L = Inf;
lb = [-L, -L, -L, -L];
ub = [L, L, L, +L];

cubic = 10000;
lb = [ECEF-cubic, -15e-3*C];
ub = [ECEF+cubic, +15e-3*C];

% process('alm')
process('eph')


function process(new_mode)
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
    % sqp_options = optimoptions('fmincon', 'Algorithm', 'interior-point');
    sqp_options.ConstraintTolerance = 1e-12;
    sqp_options.OptimalityTolerance = 1e-12;
    sqp_options.StepTolerance = 1e-12;
    sqp_options.FunctionTolerance = 1e-12;
    sqp_options.FiniteDifferenceType = 'central';
    sqp_options.Display = 'iter-detailed';
    sqp_options.MaxIterations = 50000;
    sqp_options.MaxFunctionEvaluations = 1e+6;
    sqp_options
    [result, fval, exitflag, output] = fmincon(@(params) norm(minimize_func(params, sats)), x0, [], [], [], [], lb, ub, @contrains, sqp_options);
    print_result(result, fval);
    % fprintf("расстоение от границы запрещенной зоны: %f\n", contrains(result))
    
    % fprintf("\n<strong>METHOD:</strong> Levenberg-Marquardt \n")
    % LM_options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'Display', 'none');
    % [result, resnorm, residual, exitflag, output] = lsqnonlin(@(params) minimize_func(params, sats), x0, lb, ub, LM_options);
    % print_result(result, norm(minimize_func(result, sats)));
    
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
    dists = ( sats.prMes - sqrt((sats.X - x).^2 + (sats.Y - y).^2 + (sats.Z - z).^2 ) ) - 299792458 * dt;
    % disp(norm(dists))
    % dists = mean(dists);
end
function print_result(result, fval)
    global C  ECEF
    fprintf("true coords: (%f, %f, %f) m\n R= %f \n", ECEF(1), ECEF(2), ECEF(3), sqrt(sum(ECEF.^2)))
    fprintf("computed coords: (%f, %f, %f) m, dt=%f s\n R= %f \n", result(1), result(2), result(3), result(4), sqrt(sum(result(1:3).^2)))
    fprintf("calced error, m: %f\n" , (norm(ECEF - result(1:3))))
    fprintf("function value: %f\n", fval)
    lla = MYecef2lla(result(1:3));
    fprintf("lat: %f, lon: %f, alt: %f\n", lla(1), lla(2), lla(3));
end

function [c, ceq] = contrains(x)
    global C Re H;
    % c = H - abs(sqrt(sum(x(1:3).^2)) - Re);

    lla = MYecef2lla(x(1:3));
    c = -H + abs(lla(3));

    c = abs( sqrt(sum(x(1:3).^2)) - 6371000 ) - 20000;

    ceq= [];
end


function xyz = MYlla2ecef(lla)
    % Константы
    a = 6378137.0; % Полуось WGS-84 в метрах
    f = 1 / 298.257223563; % Сжатие WGS-84
    e2 = 2 * f - f^2; % Квадрат эксцентриситета

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
    % Константы
    a = 6378137.0; % Полуось WGS-84 в метрах
    f = 1 / 298.257223563; % Сжатие WGS-84
    e2 = 2 * f - f^2; % Квадрат эксцентриситета

    x = xyz(:, 1);
    y = xyz(:, 2);
    z = xyz(:, 3);

    lon = atan2(y, x); % Долгота

    % Инициализация переменных
    p = sqrt(x.^2 + y.^2);
    lat = atan2(z, p * (1 - e2)); % Первая итерация широты
    lat_prev = lat + 1;

    while max(abs(lat - lat_prev)) > 1e-12
        lat_prev = lat;
        N = a ./ sqrt(1 - e2 * sin(lat).^2);
        alt = p ./ cos(lat) - N;
        lat = atan2(z, p * (1 - e2 * (N ./ (N + alt))));
    end

    N = a ./ sqrt(1 - e2 * sin(lat).^2);
    alt = p ./ cos(lat) - N;

    lla = [rad2deg(lat), rad2deg(lon), alt];
end
