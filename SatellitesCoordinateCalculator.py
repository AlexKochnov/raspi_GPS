from math import pi, sqrt, sin, atan2, cos
from datetime import datetime

import Constants


def check_time(time, *args, **kwargs):
    half_week = 302400.0
    if time > half_week:
        time -= 2 * half_week
    elif time < - half_week:
        time += 2 * half_week
    return time


def calc_sat_alm(ALM: list or None, time, N):
    if ALM is None or not time or not N:
        return None
    # SV_ID, week, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health, Data_ID, receiving_time = ALM
    SV_ID = ALM[0]  # ID спутника
    N0a = ALM[1]  # номер недели передаваемых данных
    Toa = ALM[2]  # опорное время внутри недели N, на которую передаются данные альманах
    e = ALM[3]  # эксцентриситет
    di = ALM[4] * pi  # rad, поправка к наклонению
    OmegaDot = ALM[5] * pi  # rad/s, скорость прецессии орбиты
    sqrtA = ALM[6]  # корень из большей полуоси
    Omega0 = ALM[7] * pi  # rad Угол восходящего узла на момент начала недели N
    omega = ALM[8] * pi  # rad аргумент перигея
    M0 = ALM[9] * pi  # rad средняя аномалия на эпоху Toa
    af0 = ALM[10]  #
    af1 = ALM[11]  #
    health = ALM[12]  #
    Data_ID = ALM[13]  #
    receiving_time: datetime = ALM[14]  # время принятия сигнала

    CORRECTION_FACTOR = 1.1
    # print(CORRECTION_FACTOR)

    OmegaEarthDot = Constants.OmegaEarthDot * CORRECTION_FACTOR

    # mu = 3.9860044 * 1e14  # m^3/s^2 гравитационная постоянная для земли WGS-84
    # OmegaEarthDot = 7.2921151467 * 10e-5  # rad/s скорость вращения земли WGS-84

    i0 = 0.30 * pi  # rad

    a = sqrtA ** 2  # большая полуось
    n0 = sqrt(Constants.mu / a ** 3)  # rad/s вычисленное среднее перемещение

    # TODO: добавить поправки генераторов
    tk = (N - N0a) * 604800 + time - Toa  # + 3600 * 6
    tk = check_time(tk)
    # print(tk)

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
    Omega_k = Omega0 + (OmegaDot - OmegaEarthDot) * tk - OmegaEarthDot * Toa
    # print(Omega_k % pi)
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


def calc_sat_eph(EPH: list or None, time, N, flag=True):
    if EPH is None or not time or not N:
        return None
    SV_ID = EPH[0]
    Noe = EPH[1]
    Toe = EPH[2]
    Toc = EPH[3]
    IODE1 = EPH[4]
    IODE2 = EPH[5]
    IODC = EPH[6]
    IDOT = EPH[7] * pi
    OmegaDot = EPH[8] * pi
    Crs = EPH[9]
    Crc = EPH[10]
    Cus = EPH[11]
    Cuc = EPH[12]
    Cis = EPH[13]
    Cic = EPH[14]
    dn = EPH[15] * pi
    i0 = EPH[16] * pi
    e = EPH[17]
    sqrtA = EPH[18]
    M0 = EPH[19] * pi
    Omega0 = EPH[20] * pi
    omega = EPH[21] * pi
    Tgd = EPH[22]
    af2 = EPH[23]
    af1 = EPH[24]
    af0 = EPH[25]
    health = EPH[26]
    accuracy = EPH[27]
    receiving_time = EPH[28]

    CORRECTION_FACTOR = 1.1
    # print(CORRECTION_FACTOR)


    OmegaEarthDot = Constants.OmegaEarthDot * CORRECTION_FACTOR

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
    Omega_k = Omega0 + (OmegaDot - OmegaEarthDot) * tk - OmegaEarthDot * Toe
    # print(Omega_k % pi)
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
