% clc
% clear all
close all

%% Константы
global C ECEF LLA
C = 299792458;

LLA = [55.690555555555555, 37.858333333333334, 140];
fprintf("lat: %f, lon: %f, alt: %f\n", LLA(1), LLA(2), LLA(3));
ECEF = lla2ecef(LLA);

%% Чтение файла
filename = 'satellites_data.txt';
opts = detectImportOptions(filename, 'Delimiter', ';');
opts.VariableNames = {'svId', 'gnssId', 'TOW', ...
    'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z', ... 
    'prMes', 'cpMes', 'elev', 'azim', ...
    'orbitSource', 'health', 'ephSource' , ... 
    'checked_eph', 'ephVal', 'healthy', 'prValid' ...
    'ura', 'ephAge', 'cno', 'used_status' ...
    };
data = readtable(filename, opts);
data.Properties.RowNames = cellstr(num2str((1:height(data))'));

% disp(data)
%% Дополнительные расчеты
data.A2E_error = round(sqrt((data.alm_x-data.eph_x).^2 + (data.alm_y-data.eph_y).^2 + (data.alm_z-data.eph_z).^2), 1);
data.eph_range = sqrt((ECEF(1)-data.eph_x).^2 + (ECEF(2)-data.eph_y).^2 + (ECEF(3)-data.eph_z).^2);
data.eph_dt = round(( - data.eph_range + data.prMes) / C, 10);
data.eph_lla = round(ecef2lla([data.eph_x, data.eph_y, data.eph_z]) .* [1, 1, 1e-3], 1);

size(data)
% disp(data)
%% Причесать талбличку и типы данных
% data = movevars(data, ['orbitSource', 'health', 'ephSource' , 'checked_eph', 'ephVal', 'healthy', 'ura', 'ephAge'], 'Before', 4);
data = data(:, [1:3, 14:28, 4:13]);
data.checked_eph = logical(data.checked_eph);
data.ephVal = logical(data.ephVal);
data.healthy = logical(data.healthy);
data.prValid = logical(data.prValid);

data.TOW = round(data.TOW);

%% Фильтрация данных
data = data(data.gnssId == 0, :);
data = data(data.orbitSource == 1 & data.health == 1 & data.ephSource == 1 ...
    & data.checked_eph & data.ephVal & data.healthy & data.prValid, :);
data(:, {'gnssId', 'orbitSource', 'health', 'ephSource' , 'checked_eph', 'ephVal', 'healthy', 'prValid'}) = [];


format long
disp(size(data));
disp(tail(data));

%% Построение задержки для каждого спутника
[uniqueSvId, ~, idx] = unique(data.svId);
colors = hsv(length(uniqueSvId));
figure; hold on;
% scatter(data.TOW, data.eph_dt, [], colors(idx, :), 'filled');
for i = 1:length(uniqueSvId)
    svId = uniqueSvId(i);
    scatter(data.TOW(data.svId == svId)/3600, data.eph_dt(data.svId == svId), [], colors(i, :), 'filled');
end
legend(arrayfun(@num2str, uniqueSvId, 'UniformOutput', false));
xlabel('TOW, ч'); ylabel('dt, сек'); title('Задержка cdt для разных спутников'); grid on;
% xlim([30, 34])

%% Отрисовка скачка псевдодальности
figure
d = data(data.svId==26, :);
hold on
% plot(d.TOW/3600, (+d.eph_dt + 0.07) * C)
plot(d.TOW/3600, d.eph_range)
plot(d.TOW/3600, d.prMes)
legend({'c*dt', 'eph range', 'pr mes'})

%% Поиск меток времени
[Tc, Tv] = groupcounts(data.TOW);
T = table(Tv, Tc, 'VariableNames', {'TOW', 'Count'});

[count, TOW_group_size] = groupcounts(Tc);
disp(table(TOW_group_size, count, 'VariableNames', {'Размер группы TOW', 'Количество групп'}));

T4 = T(T.Count >= 4, :);
T(T.Count < max(TOW_group_size), :) = [];
disp(tail(T));
if height(T) == 0
    fprintf("<strong> Все метки времени имею недостаточно спутников </strong>\n")
    return;
end

figure;
scatter(Tv/3600, Tc)
title('Количеств доступных спутников'); xlabel('TOW, ч'); ylabel('Количество');
yticks(min(Tc):max(Tc)); grid on; ylim([min(Tc) - 0.2, max(Tc) + 0.2]);


TOW = T.TOW(end);
week = 2320;

fprintf("<strong>Выбранное время TOW:</strong> %d \t(%s MSK) \n", TOW, datestr(get_msk_time(TOW, week)));

%% Спутники для нужной метки времени
fprintf("Спутники с координатами для этого времени:\n");
sats = data(data.TOW == TOW, :);
sats = sortrows(sats, 'A2E_error');
% sats = sats(1:4, :);
% sats = sats(sats.svId ~= 11, :);
disp(sats)

%% Оптимизация
L = 25e6;
x0 = [0, 0, 0, -0.008 * C];
lb = [-L, -L, -L, -0.016 * C];
ub = [L, L, L, 0.012 * C];

% fprintf("<strong>METHOD:</strong> fminsearch\n")
% result = fminsearch(@(params) norm(minimize_func(params, sats)), x0);
% print_result(result);

fprintf("\n<strong>METHOD:</strong> SQP\n")
sqp_options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none');
[result, fval, exitflag, output] = fmincon(@(params) norm(minimize_func(params, sats)), x0, [], [], [], [], lb, ub, [], sqp_options);
print_result(result);

fprintf("\n<strong>METHOD:</strong> Levenberg-Marquardt \n")
LM_options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'Display', 'none');
[result, resnorm, residual, exitflag, output] = lsqnonlin(@(params) minimize_func(params, sats), x0, lb, ub, LM_options);
print_result(result);
% 
% fprintf("\n<strong>METHOD:</strong> GA \n")
% options = optimoptions('ga', 'Display', 'off');
% [result, fval, exitflag, output] = ga(@(params) norm(minimize_func(params, sats)), 4, [], [], [], [], lb, ub, [], options);
% print_result(result);

fprintf("\n<strong>METHOD:</strong> GO - глобальная оптимизация \n")
go_options = optimoptions('fmincon', 'Algorithm', 'interior-point', 'Display', 'none');
problem = createOptimProblem('fmincon', 'objective', @(params) norm(minimize_func(params, sats)), 'x0', x0, 'lb', lb, 'ub', ub, 'options', go_options);
gs = GlobalSearch;
[result, fval, exitflag, output] = run(gs, problem);
print_result(result);

%% расчеты для графиков
time = []; x = []; y = []; z = []; cdt = []; fvals = []; counts = [];
for i = 1 : 100 : height(T4)
    t = T4.TOW(i); count = T4.Count(i);
    sats = data(data.TOW == t, :);
    [result, fval, exitflag, output] = fmincon(@(params) norm(minimize_func(params, sats)), x0, [], [], [], [], lb, ub, [], sqp_options);
    time(end+1) = t; x(end+1)=result(1); y(end+1)=result(2); z(end+1)=result(3); cdt(end+1)=result(4);
    fvals(end+1) = fval; counts(end+1) = count;
end
% disp(coords)
coords = table(time', x', y', z', cdt', fvals', counts', 'VariableNames', {'TOW', 'x', 'y', 'z', 'cdt', 'fval','count'});
coords.dt = coords.cdt / C;
coords.error = sqrt((ECEF(1)-coords.x).^2 + (ECEF(2)-coords.y).^2 + (ECEF(3)-coords.z).^2);

%% построение графиков

ms = 2; % marker size

figure; %hold on;
% subplot(3, 2, 1); plot(coords.TOW/3600, coords.x); ylabel('x, m'); xlabel('TOW, ч');
% subplot(3, 2, 3); plot(coords.TOW/3600, coords.x); ylabel('y, m'); xlabel('TOW, ч');
% subplot(3, 2, 5); plot(coords.TOW/3600, coords.x); ylabel('z, m'); xlabel('TOW, ч');
% subplot(3, 2, 2); plot(coords.TOW/3600, coords.cdt); ylabel('c*dt, m'); xlabel('TOW, ч');
U = unique(T4.Count); colors = hsv(height(U)); 
for i = 1:height(U)
    count = U(i);
    D = coords(coords.count == count, :);
    subplot(3, 2, 1); plot(D.TOW/3600, D.x, color=colors(i, :)); ylabel('x, m'); xlabel('TOW, ч'); hold on
    subplot(3, 2, 3); plot(D.TOW/3600, D.x, color=colors(i, :)); ylabel('y, m'); xlabel('TOW, ч'); hold on
    subplot(3, 2, 5); plot(D.TOW/3600, D.x, color=colors(i, :)); ylabel('z, m'); xlabel('TOW, ч'); hold on
    subplot(3, 2, 2); plot(D.TOW/3600, D.fval, color=colors(i, :)); ylabel('fval, m'); xlabel('TOW, ч'); hold on
    subplot(3, 2, 4); plot(D.TOW/3600, D.dt, color=colors(i, :)); ylabel('dt, sec'); xlabel('TOW, ч'); hold on
    subplot(3, 2, 6); plot(D.TOW/3600, D.error, color=colors(i, :)); ylabel('error, m'); xlabel('TOW, ч'); hold on
end
lgd = legend(arrayfun(@num2str, U, 'UniformOutput', false));
title(lgd, 'Counts');
sgtitle('График всех точек'); 

figure;
coords = coords(coords.error < 1e6, :);
subplot(3, 2, 1); scatter(coords.TOW/3600, coords.x, ms); ylabel('x, m'); xlabel('TOW, ч');
subplot(3, 2, 3); scatter(coords.TOW/3600, coords.x, ms); ylabel('y, m'); xlabel('TOW, ч');
subplot(3, 2, 5); scatter(coords.TOW/3600, coords.x, ms); ylabel('z, m'); xlabel('TOW, ч');
subplot(3, 2, 2); scatter(coords.TOW/3600, coords.cdt, ms); ylabel('c*dt, m'); xlabel('TOW, ч');
subplot(3, 2, 4); scatter(coords.TOW/3600, coords.dt, ms); ylabel('dt, sec'); xlabel('TOW, ч');
subplot(3, 2, 6); scatter(coords.TOW/3600, coords.error, ms); ylabel('error, m'); xlabel('TOW, ч');
sgtitle('График точек с ошибками < 1000 км');


a=0

% get_web_sats(week, TOW)


function print_result(result)
    global C  ECEF
    fprintf("calced coords: (%f, %f, %f) km, dt=%f s\n", result(1)*1e-3, result(2)*1e-3, result(3)*1e-3, result(4)/C)
    fprintf("calc error, km: %f\n" , (norm(ECEF - result(1:3))))
    lla = ecef2lla(result(1:3));
    fprintf("lat: %f, lon: %f, alt: %f\n", lla(1), lla(2), lla(3));
end


function dists = minimize_func(params, sats)
    x = params(1);  
    y = params(2);
    z = params(3);
    cdt = params(4);    
    dists = sqrt((sats.eph_x - x).^2 + (sats.eph_y - y).^2 + (sats.eph_z - z).^2) + cdt - sats.prMes;
    % dists = sqrt((sats.alm_x - x).^2 + (sats.alm_y - y).^2 + (sats.alm_z - z).^2) + cdt - sats.prMes;
end



function moscowTime = get_msk_time(tow, week)
    epoch = datetime(1980, 1, 6, 0, 0, 0); % Начальная дата GPS эпохи
    utcTime = epoch + days(7 * week) + seconds(tow);
    moscowTime = utcTime + hours(3);
end





%% Не робит

function web_sats = get_web_sats(week, TOW)
    % Загрузка TLE данных с сайта CelesTrak
    url = 'https://celestrak.com/NORAD/elements/gps-ops.txt';
    filename = 'gps-ops.txt';
    websave(filename, url);
    
    % Чтение TLE данных
    fid = fopen(filename, 'r');
    tline = fgetl(fid);
    tleData = {};
    while ischar(tline)
        tleData{end+1, 1} = tline; 
        tline = fgetl(fid);
    end
    fclose(fid);
    
    % Поиск спутников
    tleDataLs = [];
    for i = 1:length(tleData)
        if contains(tleData{i}, "GPS") && contains(tleData{i}, "PRN")
            sat = tleData(i:i+2);
            tleDataLs{end+1, 1} = sat;
        end
    end
    disp(tleDataLs)
    % Задание времени наблюдения
    startTime = datetime(2024, 6, 23, 3, 0, 0);  
    time = startTime + days(7*(week - 2320)) + seconds(TOW); 

    sats = {};
    for sat = tleDataLs
        sats(end+1, 1:4) = calc_lla_tle(sat, time);
    end
    disp(sats);
    web_sats = table(sats);
    disp(web_table)
end

function [svId, lat, lon, alt] = calc_lla_tle(tleLines, time)
    satelliteName = tleLines{1};
    line1 = tleLines{2};
    line2 = tleLines{3};

    pattern = '\(PRN (\d+)\)';
    tokens = regexp(satelliteName, pattern, 'tokens');
    svId = str2double(tokens{1}{1});

    sc = satelliteScenario(time);
    satellite(sc, tleData(i).Line1, tleData(i).Line2)
    % sat = satelliteSatellite(satellite, line1, line2);
    % [lat, lon, alt] = states(sat, time, 'CoordinateFrame', 'geographic');

    sat = earthSatellite(line1, line2);
    lla = satelliteOrbit(sat, time);
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

end