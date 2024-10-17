import numpy as np

file = open('ira_data2.txt', 'r')
data = {}
Ts = []
for line in file.readlines():
    line_elems = line.split(';')
    if len(line_elems) < 2:
        continue
    time, msg = line_elems
    time = int(np.round(float(time)))
    if len(Ts) == 0 or Ts[-1] != time:
        Ts.append(time)
        data[time] = []

    if '$PSTMSAT' in msg:
        # print(msg.split(','))
        params = msg.split(',')
        if len(params) == 7:
            header, svId, PsR, freq, X, Y, Z = params
            data[time].append([int(svId), float(X), float(Y), float(Z), float(PsR)])

# packet1 = [
#     [22575226.110000, 14290037.510000, 871783.410000, 29423321.330000],
#     [12957713.850000, 12350520.410000, 19610290.920000, 26101110.020000],
#     [6655673.790000, -13412760.690000, 21977105.770000, 29020276.740000],
#     [24570027.170000, -7014060.910000, 7230569.140000, 29426826.690000],
#     [-13120225.920000, 17180654.010000, 15632321.420000, 30135406.270000],
#     [19874819.560000, -15274145.270000, 9054147.820000, 30539832.930000],
#     [18290903.690000, 17651079.910000, -8421167.340000, 1635174.750000],
#     [15266946.360000, -16113016.350000, 14330529.740000, 5897914.250000],
#     [11495942.910000, 18959742.060000, 14916436.920000, 27278279.540000],
#     [20261863.560000, 3540201.040000, 17355071.750000, 27173805.770000],
#     [-14890943.270000, -824565.140000, 21763979.900000, 30537702.940000],
#     [-4548176.850000, 25526850.910000, 5281475.140000, 30276780.560000],
#     [-5829772.550000, 14318809.640000, 21553135.350000, 27961024.430000],
# ][:6]
# sats = packet1
#
#
# t1 = datetime.now(tz=Constants.tz_utc)
# # res = Minimizing.solve_navigation_task_LevMar(params)
# res = Minimizing.solve_navigation_task_SLSQP(packet1)
# t2 = datetime.now(tz=Constants.tz_utc)
# # res.t = t
# # res.error = np.linalg.norm(np.array(res.x[:-1]) - np.array(Constants.ECEF))
# res.dt = res.x[-1] / Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# res.calc_time = (t2 - t1).total_seconds()
# # results.append(res)
# # print(res.error)
# print(res)
# results = []
# for t in Ts:
#     sats = data[t]
#     params = [sat for sat in sats if sat[0] <= 32]
#     if len(params) >= 4 and t % 10 == 0:
#         t1 = datetime.now(tz=Constants.tz_utc)
#         # res = Minimizing.solve_navigation_task_LevMar(params)
#         res = Minimizing.solve_navigation_task_LevMar(params)
#         t2 = datetime.now(tz=Constants.tz_utc)
#         res.t = t
#         # res.error = np.linalg.norm(np.array(res.x[:-1]) - np.array(Constants.ECEF))
#         res.dt = res.x[-1] / Constants.c
#         res.lla = Transformations.ecef2lla(*res.x[:-1])
#         res.calc_time = (t2 - t1).total_seconds()
#         results.append(res)
#         print(res.error)
# a = 0
#
# ef = pd.read_csv('eph_solves.csv')
#
# XYZ = np.array([4.5e6, 2.9783e6, 3.4e6]) # = Constants.ae_glonass - 27.3m
# # XYZ = np.array([13e6, 2e6, 8e6])
# # XYZ = XYZ/np.linalg.norm(XYZ) * Constants.ae_glonass
# sats_pos = [
#     np.array([13e6, 12e6, 20e6]),
#     np.array([14e6, 16e6, 18e6]),
#     np.array([18e6, 15e6, 12e6]),
#     np.array([18.5e6, 14e6, 17e6]),
#     np.array([13e6, 20e6, 15e6]),
# ]
# # print(np.linalg.norm(XYZ))
# dt = -7.83e-3
# sats = []
# x_data = sats_pos
# y_data = []
# for sat in sats_pos:
#     pr = np.linalg.norm(sat - XYZ) + Constants.c * dt
#     y_data.append(pr)
#     sats.append(list(sat) + [pr])
#     # print(np.linalg.norm(sat))
# y_data = np.array(y_data)
# print(*sats, sep='\n')
# print(Constants.c * dt)
#
# print('lm')
# res = Minimizing.solve_navigation_task_LevMar(sats)
# res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# print(res)
#
# print('dogbox')
# res = Minimizing.solve_navigation_task_DogBox(sats)
# res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# print(res)
#
# print('trf')
# res = Minimizing.solve_navigation_task_TRF(sats)
# res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# print(res)
#
# # def func(xdata, x, y, z, cdt):
# #     xi, yi, zi = xdata[:,0], xdata[:,1], xdata[:,2]
# #     return np.sqrt((xi - x) ** 2 + (yi-y) ** 2 + (zi - z) ** 2) + cdt
# #
# # print('curve fit')
# # x, covar = curve_fit(func, x_data, y_data)
# # print(np.linalg.norm(x[:-1] - XYZ))
# # print(x[-1]/Constants.c)
# # print(x)
#
# print('sqp')
# res = Minimizing.solve_navigation_task_SLSQP(sats)#, Y0=list(XYZ) + [dt])
# res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# print(res)
# #
# print('tc') # иногда попадает с +- 13м, значение функции 0.1 (даже при НУ = итогу)
# res = Minimizing.solve_navigation_task_TC(sats)#, Y0=list(XYZ) + [dt])
# res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# print(res)
# #
# print('trf')
# res = Minimizing.solve_navigation_task_TRF(sats)
# res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# print(res)
#
# print('cobyla')
# res = Minimizing.solve_navigation_task_COBYLA(sats)
# # res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# res.lla = Transformations.ecef2lla(*res.x[:-1])
# print(res)


# xyzt0 = np.array(list(XYZ + np.array([1000, -1000, 300])) + [-8e-3*Constants.c])
# print(xyzt0 - np.array(list(XYZ) + [dt]))
# res = minimize(Minimizing.apply_func(sats, Minimizing.func), xyzt0)
# res.error = np.linalg.norm(np.array(res.x[:-1]) - XYZ)
# res.dt = res.x[-1]/Constants.c
# print(res)