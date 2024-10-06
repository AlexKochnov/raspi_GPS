from enum import Enum
from math import pi, floor, sin, cos, atan, sqrt, tan, atan2
from scipy.integrate import RK45 , solve_ivp, odeint
from scipy.integrate._ivp.ivp import OdeResult

import numpy as np
import pandas as pd

import Constants
from GNSS import GNSS, NavDataType
from TimeStamp import TimeStamp


def check_gps_time(time):
    half_week = 302400.0
    if time > half_week:
        time -= 2 * half_week
    elif time < - half_week:
        time += 2 * half_week
    return time


def calc_gps_alm(ALM: pd.DataFrame or None, time, N):
    if (ALM is None or time is None or N is None):# or
            #ALM[['week', 'Toa', 'Wdot', 'e', 'sqrtA', 'M0', 'W0', 'w']].isna().any()):
        return None
    if abs(ALM['week'] - N) > 1:
        return None
    N0a = ALM['week']
    Toa = ALM['Toa']  # опорное время внутри недели N, на которую передаются данные альманах
    e = ALM['e']  # эксцентриситет
    di = ALM['delta_i']# * pi  # rad, поправка к наклонению
    OmegaDot = ALM['Wdot']# * pi  # rad/s, скорость прецессии орбиты
    sqrtA = ALM['sqrtA']#  # корень из большей полуоси
    Omega0 = ALM['W0']# * pi  # rad Угол восходящего узла на момент начала недели N
    omega = ALM['w']# * pi  # rad аргумент перигея
    M0 = ALM['M0']# * pi  # rad средняя аномалия на эпоху Toa
    af0 = ALM['af0']
    af1 = ALM['af1']

    i0 = 0.30 * pi  # rad
    a = sqrtA ** 2  # большая полуось
    n0 = sqrt(Constants.mu / a ** 3)  # rad/s вычисленное среднее перемещение

    # TODO: добавить поправки генераторов
    tk = (N - N0a) * 604800 + time - Toa
    tk = check_gps_time(tk)
    # tk += + af0 + af1 * tk

    Mk = M0 + n0 * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    Ek = Mk  # rad
    Ek1 = Ek + 1
    while abs(Ek1 - Ek) > 1e-10:
        Ek1 = Ek
        Ek = Ek + (Mk - Ek + e * sin(Ek)) / (1 - e * cos(Ek))
    Ek = Ek1

    # nu_k = atan2(
    #     sqrt(1 - e * e) * sin(Ek),
    #     (cos(Ek) - e)
    # )
    nu_k = 2 * atan(sqrt((1.+e)/(1.-e)) * tan(Ek/2.))


    # p = a * (1 - e * e)

    # r_k = a * (1 - e * cos(Ek)) / (1 + e * cos(Ek))
    r_k = a * (1 - e * cos(Ek))
    # r_k = p / (1 + e * cos(Ek))

    ik = i0 + di
    Omega_k = Omega0 + (OmegaDot - Constants.OmegaEarthDot) * tk - Constants.OmegaEarthDot * Toa
    # Vr = sqrt(Constants.mu / p) * e * sin(nu_k)
    # Vn = sqrt(Constants.mu / p) * (1 + e * cos(nu_k))
    u_k = omega + nu_k

    X = r_k * (cos(u_k) * cos(Omega_k) - sin(u_k) * sin(Omega_k) * cos(ik))
    Y = r_k * (cos(u_k) * sin(Omega_k) + sin(u_k) * cos(Omega_k) * cos(ik))
    Z = r_k * sin(u_k) * sin(ik)


    af_dt = af0 + af1 * tk #+ Constants.F * e * sqrtA * sin(Ek)

    return af_dt, X, Y, Z

    # V0x = Vr * (cos(u_k) * cos(Omega_k) - sin(u_k) * sin(Omega_k) * cos(ik)) \
    #       - Vn * (sin(u_k) * cos(Omega_k) + cos(u_k) * sin(Omega_k) * cos(ik))
    # V0y = Vr * (cos(u_k) * sin(Omega_k) + sin(u_k) * cos(Omega_k) * cos(ik)) \
    #       - Vn * (sin(u_k) * sin(Omega_k) - cos(u_k) * cos(Omega_k) * cos(ik))
    # V0z = Vr * sin(u_k) * sin(ik) \
    #       + Vn * cos(u_k) * sin(ik)
    #
    # Vx = V0x + 0*OmegaEarthDot * Y
    # Vy = V0y - 0*OmegaEarthDot * X
    # Vz = V0z
    # return (X, Y, Z, Vx, Vy, Vz)


def calc_gps_eph(EPH: pd.DataFrame or None, time, N):
    if (EPH is None or time is None or N is None): # or (EPH[['Toe', 'IDOT', 'Wdot', 'Crs', 'Crc', 'Cus', 'Cuc', 'Cis','Cis', 'dn', 'i0', 'e', 'sqrtA', 'M0', 'W0','w']].isna().any())):
        return None
    if abs(EPH['week'] - N) > 1:
        return None
    # Noe = EPH.week
    Toc = EPH['Toc']
    # IODE1 = EPH.IODE1
    # IODE2 = EPH.IODE2
    # IODC = EPH.IODC
    Toe = EPH['Toe']
    IDOT = EPH['IDOT']# * pi
    OmegaDot = EPH['Wdot']# * pi
    Crs = EPH['Crs']
    Crc = EPH['Crc']
    Cus = EPH['Cus']
    Cuc = EPH['Cuc']
    Cis = EPH['Cis']
    Cic = EPH['Cic']
    dn = EPH['dn']# * pi
    i0 = EPH['i0']# * pi
    e = EPH['e']
    sqrtA = EPH['sqrtA']
    M0 = EPH['M0']# * pi
    Omega0 = EPH['W0']# * pi
    omega = EPH['w']# * pi
    # Tgd = EPH.Tgd
    af2 = EPH['af2']
    af1 = EPH['af1']
    af0 = EPH['af0']

    a = sqrtA ** 2  # большая полуось
    n0 = sqrt(Constants.mu / a ** 3)  # rad/s вычисленное среднее перемещение
    n = n0 + dn  # скорректированное средне движение

    tk = 0 * 604800 + time - Toe
    tk = check_gps_time(tk)
    # tk += af0 + af1 * (time-Toc) + af2 * (time-Toc)**2
    # tk += + af0 + af1 * (tk-Toc) + af2 * (tk-Toc)**2af_dt = af0 + af1 * tk + Constants.F * e * sqrtA * sin(Ek)

    Mk = M0 + n * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    Ek = Mk  # rad
    Ek1 = Ek + 1
    while abs(Ek1 - Ek) > 1e-10:
        Ek1 = Ek
        Ek = Ek + (Mk - Ek + e * sin(Ek)) / (1 - e * cos(Ek))
    Ek = Ek1

    # nu_k = atan2(
    #     sqrt(1 - e * e) * sin(Ek),
    #     (cos(Ek) - e)
    # )
    nu_k = 2 * atan(sqrt((1.+e)/(1.-e)) * tan(Ek/2.))

    Phi_k = nu_k + omega  # аргумент lat
    # r_k = a * (1 - e * cos(Ek)) / (1 + e * cos(Ek))
    r_k = a * (1 - e * cos(Ek))
    ik = i0 + IDOT * tk
    Omega_k = Omega0 + (OmegaDot - Constants.OmegaEarthDot) * tk - Constants.OmegaEarthDot * Toe
    du_k = Cuc * cos(2 * Phi_k) + Cus * sin(2 * Phi_k)
    dr_k = Crc * cos(2 * Phi_k) + Crs * sin(2 * Phi_k)
    di_k = Cic * cos(2 * Phi_k) + Cis * sin(2 * Phi_k)

    u_k = Phi_k + du_k
    r_k = r_k + dr_k
    ik = ik + di_k

    X = r_k * (cos(u_k) * cos(Omega_k) - sin(u_k) * sin(Omega_k) * cos(ik))
    Y = r_k * (cos(u_k) * sin(Omega_k) + sin(u_k) * cos(Omega_k) * cos(ik))
    Z = r_k * sin(u_k) * sin(ik)

    af_dt = af0 + af1 * (time-Toc) + af2 * (time-Toc)**2 + Constants.F * e * sqrtA * sin(Ek)

    return af_dt, X, Y, Z

    # if flag:
    #     X1, Y1, Z1, Vx1, Vy1, Vz1 = calc_sat_eph(EPH, time + 1, N, False)
    #     X0, Y0, Z0, Vx0, Vy0, Vz0 = calc_sat_eph(EPH, time - 1, N, False)
    #
    #     Vx = (X1 - X0) / 2
    #     Vy = (Y1 - Y0) / 2
    #     Vz = (Z1 - Z0) / 2
    # else:
    #     Vx, Vy, Vz = 0, 0, 0

    # return X, Y, Z, Vx, Vy, Vz


def calc_glo_eph_simple(eph, t1, N):
    X0 = eph.x
    Vx0 = eph.dx
    ddx = eph.ddx
    Y0 = eph.y
    Vy0 = eph.dy
    ddy = eph.ddy
    Z0 = eph.z
    Vz0 = eph.dz
    ddz = eph.ddz
    tb = eph.tb

    maxT = lambda ti: ti - round((ti - tb) / 86400) * 86400

    ae = Constants.ae_glonass
    GM = Constants.mu
    J20 = Constants.J20_glonass
    OmegaEarth = Constants.OmegaEarthDot

    y0 = np.array([X0, Y0, Z0, Vx0, Vy0, Vz0]) * 1000

    def func(t, y):
        X, Y, Z, Vx, Vy, Vz = y
        r = sqrt(X ** 2 + Y ** 2 + Z ** 2)
        dVx = -GM / r ** 3 * X - 1.5 * J20 * GM * ae ** 2 / r ** 5 * X * (
                    1 - 5 * Z ** 2 / r ** 2) + OmegaEarth ** 2 * X + 2 * OmegaEarth * Vy + ddx
        dVy = -GM / r ** 3 * Y - 1.5 * J20 * GM * ae ** 2 / r ** 5 * Y * (
                    1 - 5 * Z ** 2 / r ** 2) + OmegaEarth ** 2 * Y - 2 * OmegaEarth * Vx + ddy
        dVz = -GM / r ** 3 * Z - 1.5 * J20 * GM * ae ** 2 / r ** 5 * Z * (3 - 5 * Z ** 2 / r ** 2) + ddz
        # print(t, np.array([X, Y, Z, Vx, Vy, Vz, dVx, dVy, dVz]).round(0))
        return np.array([Vx, Vy, Vz, dVx, dVy, dVz])

    # runer = RK45(func, tb, y0, t1, first_step=1)
    #
    # while runer.status == 'running':
    #     runer.step()
    #     # print(runer.t, runer.y.round(3))
    # # print(runer.t, runer.y.round(3))
    # return runer.y
    res = solve_ivp(func, (tb, t1), y0, method='RK45', max_step=10, first_step=1)
    return res.y[:, -1]


def get_glo_year(N4, N):
    if 1 <= N <= 366:
        J=1
    elif 367 <= N <= 731:
        J = 2
    elif 732 <= N <= 1096:
        J = 3
    elif 1097<= N <= 1461:
        J = 4
    else:
        J = 0 # TODO - error
    return 1996 + 4 * (N4-1) + (J-1)


def get_glo_dNA(N4, NA, N):
    k = 1461 if N4 != 27 else 1460
    return N - NA - round((N-NA)/k)*k


def calc_glo_alm(alm, stamp: TimeStamp, hard=True):
    GM = Constants.mu
    ae = Constants.ae_glonass
    J20 = Constants.J20_glonass
    OmegaEath = Constants.OmegaEarthDot

    N4, N, t1 = stamp.to_glonass()

    # NA = eph.NA
    # N4 = eph.N4
    t_lambda = alm.t_lambda_n
    dT = alm.delta_T_n
    di = alm.delta_i_n
    dT_dot = alm.delta_T_dot_n
    e = alm.eps_n
    w_A = alm.omega_n
    lambda_A = alm.lambda_n
    NA = alm.NA

    # 1 - интервал прогноза dt_pr в сек
    dNa = get_glo_dNA(N4=N4, NA=NA, N=N)
    dt_pr = dNa * 86400 + (t1 - t_lambda)

    # 2 - количество целых витков W на интервале прогноза
    Tdr_avg = 43200
    W = floor(dt_pr / (Tdr_avg + dT))

    # 3 - текущее наклонение
    i_avg = 63 / 180 * pi
    i = di + i_avg

    # 4 - средний драконический период на витке W+1 и среднее движение
    Tdr = Tdr_avg + dT + (2 * W + 1) * dT_dot
    n = 2 * pi / Tdr

    # 5 - рассчитывается большая полуось последовательными приближениями (m=0, 1, 2...)
    T1 = Tdr
    a0 = -1
    a = 0
    p = 0
    while abs(a0 - a) > 1e-3:
        a0 = a
        a = ((T1 / 2 / pi) ** 2 * GM) ** (1.0 / 3.0)
        p = a * (1 - e ** 2)
        Ttt = (2 - 2.5 * sin(i) ** 2) * ((1 - e ** 2) ** 1.5 / (1 + e * cos(w_A * pi)) ** 2) + (
                    1 + e * cos(w_A * pi)) ** 3 / (1 - e ** 2)
        Tdown = 1 - 1.5 * J20 * (ae / p) ** 2 * Ttt
        T1 = Tdr / Tdown

    # 6 - текущее значение долготы восходящего узла и аргумента перигея
    lmb = lambda_A - (OmegaEath + 1.5 * J20 * n * (ae / p) ** 2 * cos(i)) * dt_pr
    w = w_A - 0.75 * J20 * n * (ae / p) ** 2 * (1 - 5 * cos(i) ** 2) * dt_pr

    # 7 - среднее значение долготы на момент прохождения восходящего узла
    E0 = -2 * atan(sqrt((1 - e) / (1 + e)) * tan(w / 2))
    L1 = w + E0 - e * sin(E0)

    # 8 - текущее значение долготы НКА
    L = L1 + n * (dt_pr - (Tdr_avg + dT) * W - dT_dot * W ** 2)
    L2 = L

    # 9 - для повышения точности, можно опустить
    def calc_derivatives(Lk):
        B = 1.5 * J20 * (ae / a) ** 2
        da_a_k = 2 * B * (1 - 1.5 * sin(i) ** 2) * (l * cos(Lk) + h * sin(Lk)) + B * sin(i) ** 2 * (
                    0.5 * h * sin(Lk) - 0.5 * l * cos(Lk) + cos(2 * Lk) + 3.5 * l * cos(3 * Lk) + 3.5 * h * sin(3 * Lk))
        da_k = da_a_k * a
        dh_k = B * (1 - 1.5 * sin(i) ** 2) * (sin(Lk) + 1.5 * l * sin(2 * Lk) - 1.5 * h * cos(2 * Lk)) - 0.25 * B * sin(
            i) ** 2 * (sin(Lk) - (7.0 / 3.0) * sin(3 * Lk) + 5 * l * sin(2 * Lk) - 8.5 * l * sin(
            4 * Lk) + 8.5 * h * cos(4 * Lk) + h * cos(2 * Lk)) + (-0.5 * B * cos(i) ** 2 * l * sin(2 * Lk))
        dl_k = B * (1 - 1.5 * sin(i) ** 2) * (cos(Lk) + 1.5 * l * cos(2 * Lk) + 1.5 * h * sin(2 * Lk)) - 0.25 * B * sin(
            i) ** 2 * (-cos(Lk) - (7.0 / 3.0) * cos(3 * Lk) - 5 * h * sin(2 * Lk) - 8.5 * l * cos(
            4 * Lk) - 8.5 * h * sin(4 * Lk) + l * cos(2 * Lk)) + 0.5 * B * cos(i) ** 2 * h * sin(2 * Lk)
        dlmb_k = -B * cos(i) * (
                    3.5 * l * sin(Lk) - 2.5 * h * cos(Lk) - 0.5 * sin(2 * Lk) - (7.0 / 6.0) * l * sin(3 * Lk) + (
                        7.0 / 6.0) * h * cos(3 * Lk))
        di_k = 0.5 * B * sin(i) * cos(i) * (
                    -l * cos(Lk) + h * sin(Lk) + cos(2 * Lk) + (7.0 / 3.0) * l * cos(3 * Lk) + (7.0 / 3.0) * h * sin(
                3 * Lk))
        dL_k = 2 * B * (1 - 1.5 * sin(i) ** 2) * (1.75 * l * sin(Lk) - 1.75 * h * cos(Lk)) + 3 * B * sin(i) ** 2 * (
                    -(7.0 / 24.0) * h * cos(Lk) - (7.0 / 24.0) * l * sin(Lk) - (49.0 / 72.0) * h * cos(3 * Lk) + (
                        49.0 / 72.0) * l * sin(3 * Lk) + 0.25 * sin(2 * Lk)) + B * cos(i) ** 2 * (
                           3.5 * l * sin(Lk) - 2.5 * h * cos(Lk) - 0.5 * sin(2 * Lk) - (7.0 / 6.0) * l * sin(3 * Lk) + (
                               7.0 / 6.0) * h * cos(3 * Lk))
        return da_k, dh_k, dl_k, dlmb_k, di_k, dL_k

    if hard:
        h = e * sin(w)
        l = e * cos(w)

        da1, dh1, dl1, dlmb1, di1, dL1 = calc_derivatives(Lk=L1)
        da2, dh2, dl2, dlmb2, di2, dL2 = calc_derivatives(Lk=L2)

        a = a + da2 - da1
        h = h + dh2 - dh1
        l = l + dl2 - dl1
        e = sqrt(h ** 2 + l ** 2)
        i = i + di2 - di1
        lmb = lmb + dlmb2 - dlmb1
        w = atan2(h, l)  # atan(h/l)
        L = L + dL2 - dL1

    # 10 - эксцентрическая аномалия
    E = 10 + E0
    while abs(E - E0) > 1e-9:
        E0 = E
        E = L - w + e * sin(E0)

    # 11 истинняа аномалия и аргумент широты
    nu = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(E / 2))
    u = nu + w

    # 12 - координаты ЦМ в прямоугольной ск
    p = a * (1 - e ** 2)
    r = p / (1 + e * cos(nu))
    X = r * (cos(lmb) * cos(u) - sin(lmb) * sin(u) * cos(i))
    Y = r * (sin(lmb) * cos(u) + cos(lmb) * sin(u) * cos(i))
    Z = r * sin(u) * sin(i)

    # Vr = sqrt(GM/p) * e * sin(nu)
    # Vu = sqrt(GM/p) * (1+e*cos(nu))
    # Vx = Vr * (cos(lmb)*cos(u)-sin(lmb)*sin(u)*cos(i)) - Vu * (cos(lmb)*sin(u)+sin(lmb)*cos(u)*cos(i)) + OmegaEath * Y
    # Vy = Vr * (sin(lmb)*cos(u)+cos(lmb)*sin(u)*cos(i)) - Vu * (sin(lmb)*sin(u)-cos(lmb)*cos(u)*cos(i)) - OmegaEath * X
    # Vz = Vr * (sin(u)*sin(i)) + Vu * (cos(u)*sin(i))

    return X, Y, Z


class RK45Solver:
    order = 5
    error_estimator_order = 4
    n_stages = 6
    C = np.array([0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1])
    A = np.array([
        [0, 0, 0, 0, 0],
        [1 / 5, 0, 0, 0, 0],
        [3 / 40, 9 / 40, 0, 0, 0],
        [44 / 45, -56 / 15, 32 / 9, 0, 0],
        [19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729, 0],
        [9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656]
    ])
    B = np.array([35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84])
    E = np.array([-71 / 57600, 0, 71 / 16695, -71 / 1920, 17253 / 339200, -22 / 525,
                  1 / 40])
    P = np.array([
        [1, -8048581381 / 2820520608, 8663915743 / 2820520608,
         -12715105075 / 11282082432],
        [0, 0, 0, 0],
        [0, 131558114200 / 32700410799, -68118460800 / 10900136933,
         87487479700 / 32700410799],
        [0, -1754552775 / 470086768, 14199869525 / 1410260304,
         -10690763975 / 1880347072],
        [0, 127303824393 / 49829197408, -318862633887 / 49829197408,
         701980252875 / 199316789632],
        [0, -282668133 / 205662961, 2019193451 / 616988883, -1453857185 / 822651844],
        [0, 40617522 / 29380423, -110615467 / 29380423, 69997945 / 29380423]])

    def __init__(self, fun, y0, t0, step=1):
        self.fun = fun
        self.t = [t0]
        self.y = [y0]
        self.f = fun(t0, y0)
        self.step = step
        self.n = y0.size
        self.K = np.empty((self.n_stages + 1, self.n), dtype=y0.dtype)

    def get(self, t):
        if t < self.t[0]:
            return None
        while t > self.t[-1]:
            self.next()
        return self.y[-1]

    def next(self):
        self.t.append(self.t[-1] + self.step)
        y, self.f = self.rk_step(self.fun, self.t[-1], self.y[-1], self.f, self.step, self.A, self.B, self.C, self.K)
        self.y.append(y)
        return self.t[-1], self.y[-1]

    @staticmethod
    def rk_step(fun, t, y, f, h, A, B, C, K):
        K[0] = f
        for s, (a, c) in enumerate(zip(A[1:], C[1:]), start=1):
            dy = np.dot(K[:s].T, a[:s]) * h
            K[s] = fun(t + c * h, y + dy)
        y_new = y + h * np.dot(K[:-1].T, B)
        f_new = fun(t + h, y_new)
        K[-1] = f_new
        return y_new, f_new


def get_glo_eph_simple_func(eph):
    X0 = eph['x']
    Vx0 = eph['dx']
    ddx = eph['ddx']
    Y0 = eph['y']
    Vy0 = eph['dy']
    ddy = eph['ddy']
    Z0 = eph['z']
    Vz0 = eph['dz']
    ddz = eph['ddz']
    tb = eph['tb']

    maxT = lambda ti: ti - round((ti - tb) / 86400) * 86400

    ae = Constants.ae_glonass
    GM = Constants.mu
    J20 = Constants.J20_glonass
    OmegaEarth = Constants.OmegaEarthDot

    y0 = np.array([X0, Y0, Z0, Vx0, Vy0, Vz0]) * 1000

    def func(t, y):
        X, Y, Z, Vx, Vy, Vz = y
        r = sqrt(X ** 2 + Y ** 2 + Z ** 2)
        dVx = -GM / r ** 3 * X - 1.5 * J20 * GM * ae ** 2 / r ** 5 * X * (
                    1 - 5 * Z ** 2 / r ** 2) + OmegaEarth ** 2 * X + 2 * OmegaEarth * Vy + ddx
        dVy = -GM / r ** 3 * Y - 1.5 * J20 * GM * ae ** 2 / r ** 5 * Y * (
                    1 - 5 * Z ** 2 / r ** 2) + OmegaEarth ** 2 * Y - 2 * OmegaEarth * Vx + ddy
        dVz = -GM / r ** 3 * Z - 1.5 * J20 * GM * ae ** 2 / r ** 5 * Z * (3 - 5 * Z ** 2 / r ** 2) + ddz
        # print(t, np.array([X, Y, Z, Vx, Vy, Vz, dVx, dVy, dVz]).round(0))
        return np.array([Vx, Vy, Vz, dVx, dVy, dVz])

    return func, y0, eph.tb




class SatellitesCoordinateCalculator:
    runer = None

    def __init__(self, data, gnssId: GNSS, mode: NavDataType):
        self.next_func = self.choose_func(data, gnssId, mode)

    def choose_func(self, data, gnssId, mode):
        match (gnssId, mode):
            case (GNSS.GPS, NavDataType.ALM):
                return lambda stamp: calc_gps_alm(data, stamp.TOW, stamp.week)
            case (GNSS.GPS, NavDataType.EPH):
                return lambda stamp: calc_gps_eph(data, stamp.TOW, stamp.week)
            case (GNSS.GLONASS, NavDataType.ALM):
                return lambda stamp: calc_glo_alm(data, stamp, hard=True)
            case (GNSS.GLONASS, NavDataType.EPH):
                self.runer = RK45Solver(*get_glo_eph_simple_func(data))
                return lambda stamp: self.runer.get(stamp.to_glonass()[-1])

    def get(self, stamp):
        return self.next_func(stamp)

