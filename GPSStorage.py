from datetime import datetime, timedelta
from math import pi, sqrt, sin, atan, atan2, cos, tan

import numpy as np
# from tabulate import tabulate

import UBXUnpacker

from GPSUtils import *
from Satellites import *
from UBXUnpacker import *


class GPSStorage:
    sat_calc_file = 'sat_raw_calc_data.txt'
    # EPH_headers = ['SV_ID', 'week', 'Toe', 'Toc', 'IODE1', 'IODE2', 'IODC', 'IDOT', 'Wdot', 'Crs', 'Crc', 'Cus', 'Cuc',
    #                'Cis', 'Cic', 'dn', 'i0', 'e', 'sqrtA', 'M0', 'W0', 'w', 'Tgd', 'af2', 'af1', 'af0',
    #                'health', 'accuracy', 'receiving_time']
    # ALM_headers = ['SV_ID', 'week', 'Toa', 'e', 'delta_i', 'Wdot', 'sqrtA', 'W0', 'w', 'M0', 'af0', 'af1',
    #                'health', 'Data_ID', 'receiving_time']

    satellites: dict[(GNSS, int): Satellite] = dict()

    start_age = datetime(1980, 1, 6)
    start_week: datetime
    week = 0
    TOW = 0
    iTOW = 0
    fTOW = 0
    time: datetime

    ecefX, ecefY, ecefZ, pAcc = 0, 0, 0, 0
    ecefVX, ecefVY, ecefVZ, sAcc = 0, 0, 0, 0
    leapS = 0
    Valid = {'TOW': 0, 'week': 0, 'leapS': 0}

    def __init__(self):
        pass

    def check_satellite(self, gnssId, svId):
        sat = (gnssId, svId)
        if sat not in self.satellites:
            self.satellites[sat] = Satellite(gnssId, svId)

    def update(self, data):
        if data is None:
            return
        if isinstance(data, Message):
            if hasattr(data, 'iTOW'):
                if self.iTOW != data.iTOW:
                    self.end_of_step()
                    pass
                self.iTOW = data.iTOW
            if isinstance(data, AID_ALM):
                self.check_satellite(GNSS.GPS, data.svId)
                self.satellites[(GNSS.GPS, data.svId)].alm = data.alm
            if isinstance(data, AID_EPH):
                self.check_satellite(GNSS.GPS, data.svId)
                self.satellites[(GNSS.GPS, data.svId)].eph = data.eph
            if isinstance(data, NAV_POSECEF):
                self.ecefX = data.ecefX
                self.ecefY = data.ecefY
                self.ecefZ = data.ecefZ
                self.pAcc = data.pAcc
            if isinstance(data, NAV_VELECEF):
                self.ecefVX = data.ecefVX
                self.ecefVY = data.ecefVY
                self.ecefVZ = data.ecefVZ
                self.sAcc = data.sAcc
            if isinstance(data, NAV_TIMEGPS):
                self.week = data.week
                self.fTOW = data.fTOW
                self.TOW = data.TOW
                self.leapS = data.leapS
                self.Valid = data.Valid
                self.start_week = self.start_age + timedelta(self.week * 7)
                self.time = self.start_week + timedelta(seconds=self.TOW / 1000)
            if isinstance(data, NAV_SAT | NAV_ORB | RXM_SVSI | RXM_RAWX):
                for (gnssId, svId), param in getattr(data, data.attr).items():
                    self.check_satellite(GNSS(gnssId), svId)
                    setattr(self.satellites[(GNSS(gnssId), svId)], data.attr, param)
            if isinstance(data, RXM_SFRBX):
                pass

    def __str__(self):
        return str(self.__dict__)


    def end_of_step(self):
        for key, satellite in self.satellites.items():
            if satellite.alm:
                alm_x, alm_y, alm_z, *_ = calc_sat_alm(satellite.alm, self.iTOW / 1000, self.week)
            else:
                alm_x, alm_y, alm_z = np.nan, np.nan, np.nan

            if satellite.eph:
                eph_x, eph_y, eph_z, *_ = calc_sat_eph(satellite.eph, self.iTOW / 1000, self.week)
            else:
                eph_x, eph_y, eph_z = np.nan, np.nan, np.nan

            if satellite.sat:
                elev = satellite.sat.elev
                azim = satellite.sat.azim
            else:
                elev, azim = np.nan, np.nan

            if satellite.rawx:
                cpMes = satellite.rawx.cpMes
                prMes = satellite.rawx.prMes
                doMes = satellite.rawx.doMes
            else:
                cpMes, prMes, doMes = np.nan, np.nan, np.nan

            self.satellites[key].sat = None
            self.satellites[key].rawx = None

            with open(self.sat_calc_file, 'a') as file:
                file.write(f"{satellite.svId};{satellite.gnssId};{self.iTOW/1000};{alm_x};{alm_y};{alm_z};{eph_x};{eph_y};{eph_z};{elev};{azim};{doMes};{cpMes};{prMes}\n")

        pass


def calc_sat_alm(ALM: list, time, N):
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

    mu = 3.9860044 * 1e14  # m^3/s^2 гравитационная постоянная для земли WGS-84
    OmegaEathDot = 7.2921151467 * 10e-5  # rad/s скорость вращения земли WGS-84
    i0 = 0.30 * pi  # rad

    a = sqrtA ** 2  # большая полуось
    n0 = sqrt(mu / a ** 3)  # rad/s вычисленное среднее перемещение

    t = time
    # TODO: добавить поправки генераторов
    tk = (N - N0a) * 604800 + time - Toa#+ 3600 * 6

    Mk = M0 + n0 * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    Ek = Mk  # rad
    for i in range(20):
        Ek = Ek + (Mk - Ek + e * sin(Ek)) / (1 - e * cos(Ek))
    nu_k = atan2(
        sqrt(1 - e * e) * sin(Ek) / (1 - e * cos(Ek)),
        (cos(Ek) - e) / (1 - e * cos(Ek))
    )
    r_k = a * (1 - e * cos(Ek))
    ik = i0 + di
    Omega_k = Omega0 + (OmegaDot - OmegaEathDot) * tk - OmegaEathDot * Toa
    p = a * (1 - e * e)
    Vr = sqrt(mu / p) * e * sin(nu_k)
    Vn = sqrt(mu / p) * (1 + e * cos(nu_k))
    u_k = omega + nu_k

    X = r_k * (cos(u_k) * cos(Omega_k) - sin(u_k) * sin(Omega_k) * cos(ik))
    Y = r_k * (cos(u_k) * sin(Omega_k) + sin(u_k) * cos(Omega_k) * cos(ik))
    Z = r_k * sin(u_k) * sin(ik)


    V0x = Vr * (cos(u_k) * cos(Omega_k) - sin(u_k) * sin(Omega_k) * cos(ik)) \
          - Vn * (sin(u_k) * cos(Omega_k) + cos(u_k) * sin(Omega_k) * cos(ik))
    V0y = Vr * (cos(u_k) * sin(Omega_k) + sin(u_k) * cos(Omega_k) * cos(ik)) \
          - Vn * (sin(u_k) * sin(Omega_k) - cos(u_k) * cos(Omega_k) * cos(ik))
    V0z = Vr * sin(u_k) * sin(ik) \
          + Vn * cos(u_k) * sin(ik)

    Vx = V0x + 0*OmegaEathDot * Y
    Vy = V0y - 0*OmegaEathDot * X
    Vz = V0z
    return (X, Y, Z, Vx, Vy, Vz)


def calc_sat_eph(EPH: list, time, N, flag=True):
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

    mu = 3.9860044 * 1e14  # m^3/s^2 гравитационная постоянная для земли WGS-84
    OmegaEathDot = 7.2921151467 * 10e-5  # rad/s скорость вращения земли WGS-84

    a = sqrtA ** 2  # большая полуось
    n0 = sqrt(mu / a ** 3)  # rad/s вычисленное среднее перемещение
    n = n0 + dn  # скорректированное средне движение
    t = time
    # TODO: добавить поправки генераторов
    tk = 0 * 604800 + time - Toe

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
    r_k = a * (1 - e * cos(Ek))
    ik = i0 + IDOT * tk
    Omega_k = Omega0 + (OmegaDot - OmegaEathDot) * tk - OmegaEathDot * Toe
    du_k = Cuc * cos(2 * Phi_k) + Cus * sin(2 * Phi_k)
    dr_k = Crc * cos(2 * Phi_k) + Crs * sin(2 * Phi_k)
    di_k = Cic * cos(2 * Phi_k) + Cis * sin(2 * Phi_k)

    u_k = Phi_k + du_k
    r_k = r_k + dr_k
    ik = ik + di_k

    X = r_k * (cos(u_k) * cos(Omega_k) - sin(u_k) * sin(Omega_k) * cos(ik))
    Y = r_k * (cos(u_k) * sin(Omega_k) + sin(u_k) * cos(Omega_k) * cos(ik))
    Z = r_k * sin(u_k) * sin(ik)

    if flag:
        X1, Y1, Z1, Vx1, Vy1, Vz1 = calc_sat_eph(EPH, time + 1, N, False)
        X0, Y0, Z0, Vx0, Vy0, Vz0 = calc_sat_eph(EPH, time - 1, N, False)

        Vx = (X1 - X0) / 2
        Vy = (Y1 - Y0) / 2
        Vz = (Z1 - Z0) / 2
    else:
        Vx, Vy, Vz = 0, 0, 0

    return X, Y, Z, Vx, Vy, Vz
