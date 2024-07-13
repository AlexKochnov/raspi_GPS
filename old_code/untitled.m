LLA = [55.690555555555555, 37.858333333333334, 140]
ECEF = lla2ecef(LLA)
TOW = 196568.000368965

eci1 = [-938794.8980826694, 3478992.4066663384, 5245214.832847871]

ss.TOW = TOW;
ss.ECEF = ECEF;
ss.eci1=eci1;
ss


minimizer = @(second) minimizing(ss, second)


options = optimoptions('ga', 'MaxGenerations', 100, 'PopulationSize', 20); 

% [x, fval] = ga(minimizer, 1, [], [], [], [], [], [], [], options); 



ecef2eci1 = @(sec) my_ecef2eci(ss.ECEF, sec + ss.TOW)




function dist = minimizing(ss, second)

    eci2 = my_ecef2eci(ss.ECEF, second + ss.TOW);
    dist = norm(ss.eci1-eci2);
end

function eci = my_ecef2eci(ecef, second)
    utc = datetime(2024, 5, 5, 0, 0, 0);
    utc = utc + seconds(second);
    eci = ecef2eci(utc, ecef);
end