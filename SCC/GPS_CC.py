from math import pi, floor, sin, cos, atan, sqrt, tan, atan2

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


def calc_gps_alm(ALM: dict or None, TOW, N) -> None or (float, np.array):
    if ALM is None or TOW is None or N is None:
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
    tk = (N - N0a) * 604800 + TOW - Toa
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

    return af_dt, np.array([X, Y, Z])

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


def calc_gps_eph(EPH: dict or None, TOW, N) -> None or (float, np.array):
    if EPH is None or TOW is None or N is None: # or (EPH[['Toe', 'IDOT', 'Wdot', 'Crs', 'Crc', 'Cus', 'Cuc', 'Cis','Cis', 'dn', 'i0', 'e', 'sqrtA', 'M0', 'W0','w']].isna().any())):
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

    tk = 0 * 604800 + TOW - Toe
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

    af_dt = af0 + af1 * (TOW-Toc) + af2 * (TOW-Toc)**2 + Constants.F * e * sqrtA * sin(Ek)

    return af_dt, np.array([X, Y, Z])

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