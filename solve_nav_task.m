% clc
% clear all
close all


%% Константы
global C ECEF LLA x0 lb ub sqp_options LM_options go_options DELAYS DELAYS2 STEP_DELAYS
C = 299792458;

LLA = [55.690555555555555, 37.858333333333334, 140];
fprintf("lat: %f, lon: %f, alt: %f\n", LLA(1), LLA(2), LLA(3));
ECEF = lla2ecef(LLA);

% STEP_DELAYS = zeros(32, 1);

% DELAYS = [];
DELAYS = zeros(32, 1);
DELAYS2 = zeros(32, 1);
load('DELAYS.mat', 'DELAYS')
% load('DELAYS2.mat', 'DELAYS2')
% disp(DELAYS)
% DELAYS = [                 NaN;                    0;    0.000848914100000;    0.000804217050000;    0.000231463950000;    0.000594201750000;    0.000317092700000;    0.000654299050000;    0.000663878650000;    0.000342306900000;   -0.000290862500000;   -0.000117447600000;    0.001058920200000;    0.000838525900000;    0.000574287700000;    0.000147726100000;    0.001087224700000;   -0.000236568600000;    0.000906455750000;    0.000768273850000;    0.000511326600000;    0.000365300700000;    0.000662466950000;   -0.000078919650000;    0.000901618300000;    0.000534093300000;    0.000380217900000;    0.000091334150000;   -0.000184972850000;    0.000031126950000;    0.000180134900000;   -0.000210540300000; ];


%% Чтение файла
% filename = 'satellites_data.txt';
data_summator_flag = false;
if ~data_summator_flag
    data = read_data('satellites_data.txt');
    week = 2321;
else
    % data1 = read_data('satellites_data1.txt'); %# другой формат
    % data2 = read_data('satellites_data2.txt'); %# другой формат
    data3 = read_data('satellites_data3.txt');
    data4 = read_data('satellites_data_4_.txt');
    data5 = read_data('satellites_data_5_.txt');
    data6 = read_data('satellites_data6.txt');
    data7 = read_data('satellites_data7.txt');
    data = vertcat(data3, data4, data5, data6, data7);
    week = 2320;

end

% data = vertcat(data2, data3, data4, data5);
data.Properties.RowNames = cellstr(num2str((1:height(data))'));
% combinedTable = vertcat(table1, table2);



%% Дополнительные расчеты
data.A2E_error = round(sqrt((data.alm_x-data.eph_x).^2 + (data.alm_y-data.eph_y).^2 + (data.alm_z-data.eph_z).^2), 1);
data.eph_range = sqrt((ECEF(1)-data.eph_x).^2 + (ECEF(2)-data.eph_y).^2 + (ECEF(3)-data.eph_z).^2);
data.eph_dt = round(( - data.eph_range + data.prMes) / C, 10);
data.eph_lla = round(ecef2lla([data.eph_x, data.eph_y, data.eph_z]) .* [1, 1, 1e-3], 1);

size(data)



%% Причесать талбличку и типы данных
% data = movevars(data, ['orbitSource', 'health', 'ephSource' , 'checked_eph', 'ephVal', 'healthy', 'ura', 'ephAge'], 'Before', 4);
data = rmmissing(data);
data = data(:, [1:3, 14:28, 4:13]);
data.checked_eph = logical(data.checked_eph);
data.ephVal = logical(data.ephVal);
data.healthy = logical(data.healthy);
data.prValid = logical(data.prValid);

data.TOW = round(data.TOW);



%% Фильтрация данных
data = data(data.gnssId == 0, :);
data = data(data.prMes > 10, :);
data = data(data.orbitSource == 1 & data.health == 1 & data.ephSource == 1 ... % & data.used_status ==1 ...
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
    % scatter(data.TOW(data.svId == svId)/3600, data.eph_dt(data.svId == svId), [], 'filled');
end
legend(arrayfun(@num2str, uniqueSvId, 'UniformOutput', false));
xlabel('TOW, ч'); ylabel('dt, сек'); grid on;
title('Задержка \Delta{t} для разных спутников'); 
subtitle('$\Delta t = (\rho_{mes}-\rho_{eph})/c$ ', 'interpreter', 'latex', 'FontSize', 12);

%% Построение задержки для каждого спутника c DELAYS
[uniqueSvId, ~, idx] = unique(data.svId);
colors = hsv(length(uniqueSvId));
figure; hold on;
% scatter(data.TOW, data.eph_dt, [], colors(idx, :), 'filled');
for i = 1:length(uniqueSvId)
    svId = uniqueSvId(i);
    scatter(data.TOW(data.svId == svId)/3600, data.eph_dt(data.svId == svId) + DELAYS(svId), [], colors(i, :), 'filled');
    % scatter(data.TOW(data.svId == svId)/3600, data.eph_dt(data.svId == svId), [], 'filled');
end
legend(arrayfun(@num2str, uniqueSvId, 'UniformOutput', false));
xlabel('TOW, ч'); ylabel('dt, сек'); grid on;
title('Задержка \Delta{t} для разных спутников с учетом несинхронизированности'); 
subtitle('$\Delta t = (\rho_{mes}-\rho_{eph})/c$ ', 'interpreter', 'latex', 'FontSize', 12);


% xlim([30, 34])
%
if false
    if true
        SVs = unique(data.svId);
        SVs = setdiff(SVs, 1, 'stable');
        N = 5;
        legs = [];
        figure
        hold on
        % SV1s = [5, 2, 4, 10, 14, 24,];
        % SV2s = [13, 3, 6, 15, 17, 32,];
        SV1s = [2  2  2  5  5  5  8 15 16 16];
        SV2s = [8 14 22 13 15 20 27 23 26 27];

        % delays = table([], [], [], VariableNames ={'from', 'to', 'ddt'});
        delays = cell(32, 32);
        delays_list = [];

        for i = 1:32%length(SV1s)
            % sv1 = SV1s(i); sv2 = SV2s(i);
            for j = i+1:32
                sv1=i; sv2=j;
                merged_data = get_merged_data(data, sv1, sv2);

                % filtered = filterMean(merged_data);
                % ddt = median(filtered.eph_dt_diff);
                % H = height(filtered)
                ddt, H = most_common_ddt(merged_data);
                delays{i, j} = ddt;
                % delays{j, i} = -ddt;
                if ~isnan(ddt)
                    delays_list = [delays_list; [i, j, ddt, H]];
                    delays_list = [delays_list; [j, i, -ddt, H]];
                end
                if height(merged_data) < 10000
                    continue
                end
                scatter(merged_data.TOW/3600, 1000*merged_data.eph_dt_diff, 150, '.');
                legs = [legs, sprintf("sv %d - sv %d", sv1, sv2)];
                % fprintf("%d & %d: %d\n", sv1, sv2, height(merged_data));
            end
        end
        legend(legs);
        xlabel('TOW, ч');
        ylabel('\Delta t_1 - \Delta t_2, мс')
        grid on;
    end
    %------------------------------------------------------------------
    % figure
    % mg = get_merged_data(data, 5, 13);
    % avg = sum(mg.eph_dt_diff) / height(mg);
    % % filtered = filterQ13(mg);
    % filtered = filterMean(mg);
    % % scatter(mg.TOW/3600, 1000*(mg.eph_dt_diff-avg) * 1000, '.');
    % scatter(filtered.TOW/3600, (filtered.eph_dt_diff) * 1000, '.');
    % xlabel('TOW, ч');
    % ylabel('\Delta t_1 - \Delta t_2, мс')
    % grid on;

    DELAYS = zeros(32, 1);
    for i = 1:32
        DELAYS(i) = find_delay(delays_list, 2, i);
    end
    disp(DELAYS)
end
% %%
% DELAYS = zeros(32, 1);
% for i = 1:32
%     DELAYS(i) = find_delay(delays_list, 2, i);
% end
% disp(DELAYS)

%%
% disp(delays_list)
% finder = @find_delay;
% sv1 = 2;
% sv2 = 5;
% D = find_delay(delays_list, sv1, sv2);
% fprintf("%d -> %d = %f\n", sv1, sv2, D);
% 

%% Отрисовка скачка псевдодальности
figure
sv = 21;
d = data(data.svId==sv, :);
hold on
% plot(d.TOW/3600, (+d.eph_dt + 0.07) * C)
scatter(d.TOW/3600, d.eph_range, '.')
scatter(d.TOW/3600, d.prMes, '.')
grid on; xlabel('TOW, ч'); ylabel('\rho, m');
legend({'eph range', 'pr mes'})
str = sprintf('Дальности до спутника %d', sv);
title(str)



%% Поиск меток времени
[Tc, Tv] = groupcounts(data.TOW);
T = table(Tv, Tc, 'VariableNames', {'TOW', 'Count'});

[count, TOW_group_size] = groupcounts(Tc);
disp(table(TOW_group_size, count, 'VariableNames', {'Размер группы TOW', 'Количество групп'}));

T4 = T(T.Count >= 4, :);
T(T.Count < min(7, max(TOW_group_size)), :) = [];
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
% TOW = T.TOW(500);

fprintf("<strong>Выбранное время TOW:</strong> %d \t(%s MSK) \n", TOW, datestr(get_msk_time(TOW, week)));



%% Спутники для нужной метки времени
fprintf("Спутники с координатами для этого времени:\n");
sats = data(data.TOW == TOW, :);
sats = sortrows(sats, 'A2E_error');
% sats = sats(1:4, :);
% sats = sats(sats.svId ~= 21, :);
disp(sats)

%%
if height(sats(sats.svId == 2, :))
    line = sats(sats.svId == 2, :);
    dt2 = line.eph_dt;
    for i = 1: height(sats)
        sat = sats(i, :);
        fprintf("%d: %f | %f | %f\n", sat.svId, DELAYS(sat.svId), DELAYS2(sat.svId), dt2 - sat.eph_dt)
    end
end

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






%% построение графиков
% load('coords1.dat', 'coords')

coords = calc_coords(data, T4, 100);
% draw_coords(coords, T4, 'График всех точек', true);
% draw_coords(coords(coords.error < 1e6, :), T4, 'График точек с ошибками < 1000 км', true);

draw_coords(coords, T4, 'График всех точек', false);
draw_coords(coords(coords.error < 200e3, :), T4, 'График точек с ошибками < 200 км', false);
draw_coords(coords(coords.error < 5e3, :), T4, 'График точек с ошибками < 5 км', false);
%%
% coords2 = calc_coords(data, T, 1);
% draw_coords(coords2, T, 'График всех точек', false);
% draw_coords(coords2(coords2.error < 1e6, :), T, 'График точек с ошибками < 1000 км', false);

%% 



% get_web_sats(week, TOW)


%%


%% расчеты для графиков
function coords = calc_coords(data, T, step)
    global ECEF C x0 lb ub sqp_options LM_options
    time = []; x = []; y = []; z = []; cdt = []; fvals = []; counts = [];
    for i = 1 : step : height(T)
        t = T.TOW(i); count = T.Count(i);
        sats = data(data.TOW == t, :);
        % [result, fval, exitflag, output] = fmincon(@(params) norm(minimize_func(params, sats)), x0, [], [], [], [], lb, ub, [], sqp_options);
        [result, resnorm, residual, exitflag, output] = lsqnonlin(@(params) minimize_func(params, sats), x0, lb, ub, LM_options);
        fval = resnorm;

        time(end+1) = t; x(end+1)=result(1); y(end+1)=result(2); z(end+1)=result(3); cdt(end+1)=result(4);
        fvals(end+1) = fval; counts(end+1) = count;
    end
    % disp(coords)
    coords = table(time', x', y', z', cdt', fvals', counts', 'VariableNames', {'TOW', 'x', 'y', 'z', 'cdt', 'fval','count'});
    coords.dt = coords.cdt / C;
    coords.error = sqrt((ECEF(1)-coords.x).^2 + (ECEF(2)-coords.y).^2 + (ECEF(3)-coords.z).^2);
    lla = ecef2lla([coords.x, coords.y, coords.z]);
    coords.lat = lla(:,1); coords.lon = lla(:,2); coords.alt = lla(:,3);
end




%% Чтение таблицы
function data = read_data(filename)
    opts = detectImportOptions(filename, 'Delimiter', ';');
    opts.VariableNames = {'svId', 'gnssId', 'TOW', ...
        'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z', ... 
        'prMes', 'cpMes', 'elev', 'azim', ...
        'orbitSource', 'health', 'ephSource' , ... 
        'checked_eph', 'ephVal', 'healthy', 'prValid' ...
        'ura', 'ephAge', 'cno', 'used_status' ...
        };
    data = readtable(filename, opts);
end



%% Вывод
function print_result(result)
    global C  ECEF
    fprintf("calced coords: (%f, %f, %f) km, dt=%f s\n", result(1)*1e-3, result(2)*1e-3, result(3)*1e-3, result(4)/C)
    fprintf("calc error, m: %f\n" , (norm(ECEF - result(1:3))))
    lla = ecef2lla(result(1:3));
    fprintf("lat: %f, lon: %f, alt: %f\n", lla(1), lla(2), lla(3));
end

function ddts = get_ddt_global(sats)
    global DELAYS DELAYS2
    ddts= zeros(height(sats), 1);
    for i = 1:height(sats)
        sat = sats(i, :);
        svId = sat.svId;
        ddts(i) = DELAYS(svId);
    end
end

function ddts = get_ddt_local(sats)
    % global DELAYS
    ddts= zeros(height(sats), 1);
    sv1 = sats(1, :);
    for i = 2:height(sats)
        sat = sats(i, :);
        % svId = sat.svId;
        ddts(i) = round(sv1.eph_dt - sat.eph_dt, 6);
    end
end

function dists = minimize_func(params, sats)
    global C
    x = params(1);  
    y = params(2);
    z = params(3);
    cdt = params(4);    
    dists = sqrt((sats.eph_x - x).^2 + (sats.eph_y - y).^2 + (sats.eph_z - z).^2) + (cdt - C * get_ddt_global(sats)) - sats.prMes;
    % dists = sqrt((sats.alm_x - x).^2 + (sats.alm_y - y).^2 + (sats.alm_z - z).^2) + cdt - sats.prMes;
end



%% преобразование времени
function moscowTime = get_msk_time(tow, week)
    epoch = datetime(1980, 1, 6, 0, 0, 0); % Начальная дата GPS эпохи
    utcTime = epoch + days(7 * week) + seconds(tow);
    moscowTime = utcTime + hours(3);
end



%% рисовашка
function draw_coords(coords, T, name, xyz_flag)
    global ECEF LLA
    ms = 3; % marker size
    figure; %hold on;
    U = unique(T.Count); 
    colors = hsv(height(U)); 
    for i = 1:height(U)
        count = U(i);
        % if count == 6
        ms = count + 1;
        % end
        % subplot(3, 2, 1); plot(coords.TOW/3600, coords.x); ylabel('x, m'); xlabel('TOW, ч');
    % subplot(3, 2, 3); plot(coords.TOW/3600, coords.x); ylabel('y, m'); xlabel('TOW, ч');
    % subplot(3, 2, 5); plot(coords.TOW/3600, coords.x); ylabel('z, m'); xlabel('TOW, ч');
    % subplot(3, 2, 2); plot(coords.TOW/3600, coords.cdt); ylabel('c*dt, m'); xlabel('TOW, ч');
        D = coords(coords.count == count, :);
        if xyz_flag
            subplot(3, 2, 1); scatter(D.TOW/3600, D.x, ms, colors(i, :), "filled"); 
            ylabel('x, m'); xlabel('TOW, ч'); hold on; grid on;
            subplot(3, 2, 3); scatter(D.TOW/3600, D.y, ms, colors(i, :), "filled");
            ylabel('y, m'); xlabel('TOW, ч'); hold on; grid on;
            subplot(3, 2, 5); scatter(D.TOW/3600, D.z, ms, colors(i, :), "filled"); 
            ylabel('z, m'); xlabel('TOW, ч'); hold on; grid on;
        else
            subplot(3, 2, 1); scatter(D.TOW/3600, D.lat, ms, colors(i, :), "filled"); 
            ylabel('lat, °'); xlabel('TOW, ч'); hold on; grid on;
            subplot(3, 2, 3); scatter(D.TOW/3600, D.lon, ms, colors(i, :), "filled");
            ylabel('lon, °'); xlabel('TOW, ч'); hold on; grid on;
            subplot(3, 2, 5); scatter(D.TOW/3600, D.alt, ms, colors(i, :), "filled"); 
            ylabel('alt, m'); xlabel('TOW, ч'); hold on; grid on;
        end
        subplot(3, 2, 2); scatter(D.TOW/3600, D.fval, ms, colors(i, :), "filled"); 
        ylabel('fval, m'); xlabel('TOW, ч'); hold on; grid on; ylim([0, 0.5e6]);
        subplot(3, 2, 4); scatter(D.TOW/3600, D.dt, ms, colors(i, :), "filled"); 
        ylabel('dt, sec'); xlabel('TOW, ч'); hold on; grid on;
        subplot(3, 2, 6); scatter(D.TOW/3600, D.error / 1e3, ms, colors(i, :), "filled"); 
        ylabel('error, km'); xlabel('TOW, ч'); hold on; grid on;
    end
    if xyz_flag
        true_coords = ECEF;
    else
        true_coords = LLA;
    end
    for i = 1:3
        subplot(3, 2, 2*i-1); 
        plot([min(coords.TOW/3600), max(coords.TOW/3600)], [true_coords(i), true_coords(i)], color='black', linestyle='--', linewidth=2);
    end
    lgd = legend(arrayfun(@num2str, U, 'UniformOutput', false));
    title(lgd, 'Counts');
    sgtitle(name); 

end
% ms = 3;
% figure;
% coords = coords(coords.error < 1e6, :);
% subplot(3, 2, 1); scatter(coords.TOW/3600, coords.x, ms); ylabel('x, m'); xlabel('TOW, ч');
% subplot(3, 2, 3); scatter(coords.TOW/3600, coords.x, ms); ylabel('y, m'); xlabel('TOW, ч');
% subplot(3, 2, 5); scatter(coords.TOW/3600, coords.x, ms); ylabel('z, m'); xlabel('TOW, ч');
% subplot(3, 2, 2); scatter(coords.TOW/3600, coords.cdt, ms); ylabel('c*dt, m'); xlabel('TOW, ч');
% subplot(3, 2, 4); scatter(coords.TOW/3600, coords.dt, ms); ylabel('dt, sec'); xlabel('TOW, ч');
% subplot(3, 2, 6); scatter(coords.TOW/3600, coords.error, ms); ylabel('error, m'); xlabel('TOW, ч');
% sgtitle('График точек с ошибками < 1000 км');

%%

function merged_data = get_merged_data(data, sv1, sv2)
    data_sv1 = data(data.svId == sv1, :);
    data_sv2 = data(data.svId == sv2, :);
    % merged_data = outerjoin(data_sv1, data_sv2, 'Keys', 'TOW', 'MergeKeys', true, ...
    % 'RightVariables', {'eph_dt'}, 'RightVariablesSuffix', '_sv2', 'LeftVariables', {'eph_dt'});
    % merged_data = rmmissing(merged_data);
    merged_data = outerjoin(data_sv1, data_sv2, 'Keys', 'TOW', 'MergeKeys', true);
    merged_data = merged_data(~isnan(merged_data.eph_dt_data_sv1) & ~isnan(merged_data.eph_dt_data_sv2), :);
    merged_data.eph_dt_diff = merged_data.eph_dt_data_sv1 - merged_data.eph_dt_data_sv2;
    
end

function filtered_data = filterQ13(merged_data)
    Q1 = quantile(merged_data.eph_dt_diff, 0.25);
    Q3 = quantile(merged_data.eph_dt_diff, 0.75);
    IQR = Q3 - Q1;
    lower_bound = Q1 - 1.5 * IQR;
    upper_bound = Q3 + 1.5 * IQR;
    filtered_data = merged_data(merged_data.eph_dt_diff >= lower_bound & merged_data.eph_dt_diff <= upper_bound, :);
end

function filtered_data = filterMean(merged_data)
    threshold = 3*1e-6;
    median_dt_diff = median(merged_data.eph_dt_diff);
    filtered_data = merged_data(abs(merged_data.eph_dt_diff - median_dt_diff) <= threshold, :);
end

function [most_frequent_value, occurrences] = most_common_ddt(merged_data)
    most_frequent_value = mode(merged_data.eph_dt_diff);
    occurrences = sum(merged_data.eph_dt_diff == most_frequent_value);
end

function line = delay_line(from, to, ddt) 
    table(from, to, ddt, VariableNames ={'from', 'to', 'ddt'})
end

function delay = find_delay(delays_list, sv1, sv2)
    Vis = zeros(32,1);
    if sv2 == 1
        delay = 0; % вместо NAN
        return
    end
    delay = dfs_find_delay(delays_list, Vis, sv1, sv2);
end

function delay = dfs_find_delay(delays_list, Vis, sv1, sv2)
    fprintf("Find %d -> %d\n", sv1, sv2)
    delay = 0;
    Vis(sv1) = true;
    if sv1 == sv2
        return
    end
    % neighbourhouds = delays_list(delays_list(:,1) == sv1, :);
    neighbourhoods = delays_list(delays_list(:,1) == sv1, :);

    for i = 1 : height(neighbourhoods)
        line = neighbourhoods(i, :);
        next = line(2);
        if Vis(next) == true
            continue
        end
        delay = dfs_find_delay(delays_list, Vis, next, sv2);
        if ~isnan(delay)
            delay = delay + line(3);
            return
        end
    end
    delay = NaN;
    % Vis(sv1) = false; %% для обхода с max(counts)
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