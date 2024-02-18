from datetime import datetime, timedelta
from math import pi, sqrt, sin, atan, atan2, cos, tan

import numpy as np
# from tabulate import tabulate

import UBXUnpacker

from GPSUtils import *
from Satellites import *
from UBXUnpacker import *


class GPSStorage:
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


def calc_sat_alm(ALM: list, t):
    # SV_ID, week, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health, Data_ID, receiving_time = ALM
    SV_ID = ALM[0]  # ID спутника
    week = ALM[1]  # номер недели передаваемых данных
    Toa = ALM[2]  # опорное время внутри недели N, на которую передаются данные альманах
    e = ALM[3]  # эксцентриситет
    di = ALM[4] * pi  # rad, поправка к наклонению
    Wdot = ALM[5] * pi  # rad/s, скорость прецессии орбиты
    sqrtA = ALM[6]  # корень из большей полуоси
    W0 = ALM[7] * pi  # rad Угол восходящего узла на момент начала недели N
    w = ALM[8] * pi  # rad аргумент перигея
    M0 = ALM[9] * pi  # rad средняя аномалия на эпоху Toa
    af0 = ALM[10]  #
    af1 = ALM[11]  #
    health = ALM[12]  #
    Data_ID = ALM[13]  #
    receiving_time: datetime = ALM[14]  # время принятия сигнала

    mu = 3.986005 * 1e14  # m^3/s^2 гравитационная постоянная для земли WGS-84
    Wedot = 7.2921151467 * 10e-5  # rad/s скорость вращения земли WGS-84
    i0 = 0.30 * pi  # rad

    A = sqrtA ** 2  # большая полуось
    n0 = sqrt(mu / A ** 3)  # rad/s вычисленное среднее перемещение
    # tk = t - Toa  # время от эпохи отсчета эфимерид
    # if tk > 302400:
    #     tk -= 604800
    # elif tk < 302400:
    #     tk += 604800

    tk = 0

    n = n0  # скорректированное средне движение
    Mk = M0 + n * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    E0 = Mk  # rad
    for i in range(10):
        E1 = E0 + (Mk - E0 + e * sin(E0)) / (1 - e * cos(E0))
        E0 = E1
    E = E1  # rad
    v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(E / 2))  # истиная аномалия
    u = v + w  # аргумент lat
    r = A * (1 - e * cos(E))  # скорректированный радиус
    i = i0 + di  # скорректированный наклон
    x1 = r * cos(u)  # позиция в плоскости орбиты
    y1 = r * sin(u)  # позиция в плоскости орбиты
    W = W0 + (Wdot - Wedot) * tk - Wedot * Toa  # исправленная долгота lon восходящего узла
    x = x1 * cos(W) - y1 * cos(i) * sin(W)
    y = x1 * sin(W) + y1 * cos(i) * cos(W)
    z = y1 * sin(i)

    return u, W, i, Mk, r, x, y, z


def calc_sat_eph(EPH: list, t):
    SV_ID = EPH[0]
    week = EPH[1]
    Toe = EPH[2]
    Toc = EPH[3]
    IODE1 = EPH[4]
    IODE2 = EPH[5]
    IODC = EPH[6]
    IDOT = EPH[7] * pi
    Wdot = EPH[8] * pi
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
    W0 = EPH[20] * pi
    w = EPH[21] * pi
    Tgd = EPH[22]
    af2 = EPH[23]
    af1 = EPH[24]
    af0 = EPH[25]
    health = EPH[26]
    accuracy = EPH[27]
    receiving_time = EPH[28]

    mu = 3.986005 * 1e14  # m^3/s^2 гравитационная постоянная для земли WGS-84
    Wedot = 7.2921151467 * 10e-5  # rad/s скорость вращения земли WGS-84

    # dTsv = af0 + af
    # t = Toe

    A = sqrtA ** 2  # большая полуось
    n0 = sqrt(mu / A ** 3)  # rad/s вычисленное среднее перемещение
    tk = t - Toe  # время от эпохи отсчета эфимерид
    if tk > 302400:
        tk -= 604800
    elif tk < 302400:
        tk += 604800
    n = n0 + dn  # скорректированное средне движение
    Mk = M0 + n * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    E0 = Mk  # rad
    for i in range(10):
        E1 = E0 + (Mk - E0 + e * sin(E0)) / (1 - e * cos(E0))
        E0 = E1
    E = E1  # rad
    # v = 2 * math.atan(sqrt((1+e)/(1-e)) * tan(E/2)) # истиная аномалия
    v = atan2(
        sqrt(1 - e * e) * sin(E),
        cos(E) - e)
    Phi = v + w  # аргумент lat
    du = Cus * sin(2 * Phi) + Cuc * cos(2 * Phi)  # коррекция lat
    dr = Crs * sin(2 * Phi) + Crc * cos(2 * Phi)  # коррекция радиуса
    di = Cis * sin(2 * Phi) + Cic * cos(2 * Phi)  # коррекция наклона
    u = Phi + du  # скорректированный агрумент lat
    r = A * (1 - e * cos(E)) + dr  # скорректированный радиус
    i = i0 + di + IDOT * tk  # скорректированный наклон
    x1 = r * cos(u)  # позиция в плоскости орбиты
    y1 = r * sin(u)  # позиция в плоскости орбиты
    W = W0 + (Wdot - Wedot) * tk - Wedot * Toe  # исправленная долгота lon восходящего узла
    x = x1 * cos(W) - y1 * cos(i) * sin(W)
    y = x1 * sin(W) + y1 * cos(i) * cos(W)
    z = y1 * sin(i)

    return u, W, i, Mk, r, x, y, z
    # u - скорректированный агрумент lat
    # i - скорректированный наклон
    # W - исправленная долгота lon восходящего узла
