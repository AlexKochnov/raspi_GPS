from math import pi, floor, sin, cos, atan, sqrt, tan, atan2

import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp

import Constants
from GNSS import GNSS, NavDataType
from TimeStamp import TimeStamp


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


def calc_glo_eph_simple(eph, t1, NA, N4):
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

    # runer = RK45(func, tb, y0, t1, first_step=1)
    #
    # while runer.status == 'running':
    #     runer.step()
    #     # print(runer.t, runer.y.round(3))
    # # print(runer.t, runer.y.round(3))
    # return runer.y
    res = solve_ivp(func, (tb, t1), y0, method='RK45', max_step=10, first_step=1)
    af_dt = eph['tau'] - eph['gamma'] * (t1 - tb)
    return af_dt, res.y[:, -1]


def calc_glo_alm(alm, t1, NA, N4, hard=True):
    GM = Constants.mu
    ae = Constants.ae_glonass
    J20 = Constants.J20_glonass
    OmegaEath = Constants.OmegaEarthDot

    # N4, N, t1 = stamp.to_glonass()

    # NA = eph.NA
    # N4 = eph.N4
    t_lambda = alm['t_lambda_n']
    dT = alm['delta_T_n']
    di = alm['delta_i_n']
    dT_dot = alm['delta_T_dot_n']
    e = alm['eps_n']
    w_A = alm['omega_n']
    lambda_A = alm['lambda_n']
    NA = alm['NA']

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

    af_dt = alm['tau_n']
    return af_dt, np.array([X, Y, Z])


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

    return func, y0, eph['tb']


