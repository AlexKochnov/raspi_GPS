from math import pi, sqrt, sin, atan2, cos
import pandas as pd

import Constants


def check_time(time):
    half_week = 302400.0
    if time > half_week:
        time -= 2 * half_week
    elif time < - half_week:
        time += 2 * half_week
    return time


def calc_sat_alm(ALM: pd.DataFrame or None, time, N):
    if ALM is None or time is None or N is None:
        return None
    N0a = ALM.week
    Toa = ALM.Toa  # опорное время внутри недели N, на которую передаются данные альманах
    e = ALM.e  # эксцентриситет
    di = ALM.delta_i * pi  # rad, поправка к наклонению
    OmegaDot = ALM.Wdot * pi  # rad/s, скорость прецессии орбиты
    sqrtA = ALM.sqrtA  # корень из большей полуоси
    Omega0 = ALM.W0 * pi  # rad Угол восходящего узла на момент начала недели N
    omega = ALM.w * pi  # rad аргумент перигея
    M0 = ALM.M0 * pi  # rad средняя аномалия на эпоху Toa
    # af0 = ALM.af0
    # af1 = ALM.af1

    i0 = 0.30 * pi  # rad
    a = sqrtA ** 2  # большая полуось
    n0 = sqrt(Constants.mu / a ** 3)  # rad/s вычисленное среднее перемещение

    # TODO: добавить поправки генераторов
    tk = (N - N0a) * 604800 + time - Toa  # + 3600 * 6
    tk = check_time(tk)

    Mk = M0 + n0 * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    Ek = Mk  # rad
    for i in range(20):
        Ek = Ek + (Mk - Ek + e * sin(Ek)) / (1 - e * cos(Ek))
    nu_k = atan2(
        sqrt(1 - e * e) * sin(Ek) / (1 - e * cos(Ek)),
        (cos(Ek) - e) / (1 - e * cos(Ek))
    )
    # r_k = a * (1 - e * cos(Ek)) / (1 + e * cos(Ek))
    r_k = a * (1 - e * cos(Ek))
    ik = i0 + di
    Omega_k = Omega0 + (OmegaDot - Constants.OmegaEarthDot) * tk - Constants.OmegaEarthDot * Toa
    p = a * (1 - e * e)
    Vr = sqrt(Constants.mu / p) * e * sin(nu_k)
    Vn = sqrt(Constants.mu / p) * (1 + e * cos(nu_k))
    u_k = omega + nu_k

    X = r_k * (cos(u_k) * cos(Omega_k) - sin(u_k) * sin(Omega_k) * cos(ik))
    Y = r_k * (cos(u_k) * sin(Omega_k) + sin(u_k) * cos(Omega_k) * cos(ik))
    Z = r_k * sin(u_k) * sin(ik)

    return X, Y, Z

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


def calc_sat_eph(EPH: pd.DataFrame or None, time, N):
    if EPH is None or time is None or N is None:
        return None
    # Noe = EPH.week
    Toe = EPH.Toe
    # Toc = EPH.Toc
    # IODE1 = EPH.IODE1
    # IODE2 = EPH.IODE2
    # IODC = EPH.IODC
    IDOT = EPH.IDOT * pi
    OmegaDot = EPH.Wdot * pi
    Crs = EPH.Crs
    Crc = EPH.Crc
    Cus = EPH.Cus
    Cuc = EPH.Cuc
    Cis = EPH.Cis
    Cic = EPH.Cic
    dn = EPH.dn * pi
    i0 = EPH.i0 * pi
    e = EPH.e
    sqrtA = EPH.sqrtA
    M0 = EPH.M0 * pi
    Omega0 = EPH.W0 * pi
    omega = EPH.w * pi
    # Tgd = EPH.Tgd
    # af2 = EPH.af2
    # af1 = EPH.af1
    # af0 = EPH.af0

    a = sqrtA ** 2  # большая полуось
    n0 = sqrt(Constants.mu / a ** 3)  # rad/s вычисленное среднее перемещение
    n = n0 + dn  # скорректированное средне движение

    # TODO: добавить поправки генераторов
    # dt = check_time(time - Toc)
    # satNr = (af2 * dt + af1) * dt + af0 - Tgd
    # time = time - satNr
    # tk = check_time(time - Toe)

    tk = 0 * 604800 + time - Toe
    tk = check_time(tk)
    # print(tk)

    Mk = M0 + n * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    Ek = Mk  # rad
    for i in range(20):
        Ek = Ek + (Mk - Ek + e * sin(Ek)) / (1 - e * cos(Ek))

    nu_k = atan2(
        sqrt(1 - e * e) * sin(Ek) / (1 - e * cos(Ek)),
        (cos(Ek) - e) / (1 - e * cos(Ek))
    )
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
    return X, Y, Z

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
