from datetime import datetime, timedelta
from math import pi, sqrt, sin, atan, atan2, cos, tan

import numpy as np
# from tabulate import tabulate

from UBXUnpacker import MessageTypes

from GPSUtils import *


class GPSStorage:
    EPH: list = [None]*33
    EPH_headers = ['SV_ID', 'week', 'Toe', 'Toc', 'IODE1', 'IODE2', 'IODC', 'IDOT', 'Wdot', 'Crs', 'Crc', 'Cus', 'Cuc',
                   'Cis', 'Cic', 'dn', 'i0', 'e', 'sqrtA', 'M0', 'W0', 'w', 'Tgd', 'af2', 'af1', 'af0',
                   'health', 'accuracy', 'receiving_time']
    ALM: list = [None]*33
    ALM_headers = ['SV_ID', 'week', 'Toa', 'e', 'delta_i', 'Wdot', 'sqrtA', 'W0', 'w', 'M0', 'af0', 'af1',
                   'health', 'Data_ID', 'receiving_time']
    NAV: list = []
    NAV_headers = []

    start_age = datetime(1980, 1, 6)
    start_week: datetime
    week = 0
    TOW = 0
    time: datetime

    def __init__(self):
        self.EPH[0] = self.EPH_headers
        self.ALM[0] = self.ALM_headers
        # TODO: добавить общую навигацию NAV
        pass

    def update(self, type, data):
        if data is None:
            return
        match type:
            case MessageTypes.EPH:
                self.update_EPH(data)
            case MessageTypes.ALM:
                self.update_ALM(data)
            case MessageTypes.NAV:
                if data.type == 'NAV_TIMEGPS':
                    self.update_NAV_TIMEGPS(data)

        pass

    def update_NAV_TIMEGPS(self, data):
        self.TOW = data.TOW
        self.week = data.week
        self.start_week = self.start_age + timedelta(self.week * 7)
        self.time = self.start_week + timedelta(seconds=self.TOW / 1000)

    def update_EPH(self, data):
        SV_ID = data[0]
        if self.EPH[SV_ID] != data:
            with open('eph_long.txt', 'a') as eph:
                eph.write(str(data) + '\n')
            self.EPH[SV_ID] = data
        # sat = calc_sat_eph(data, t)
        # print(f'{data[0]}, {np.array(sat[:4]) * 180 / pi % 360}', {sat[4:]}, )

    def update_ALM(self, data):
        SV_ID = data[0]
        if self.ALM[SV_ID] != data:
            with open('alm_long.txt', 'a') as eph:
                eph.write(str(data) + '\n')
            self.ALM[SV_ID] = data
        # sat = calc_sat_alm(data, t)
        # print(f'{data[0]}, {np.array(sat[:4]) * 180 / pi % 360}', {sat[4:]}, )

    def __str__(self):
        EPH = [sat for sat in self.EPH[1:] if sat]
        ALM = [sat for sat in self.ALM[1:] if sat]
        return str(EPH) + '\n\n' + str(ALM)
        # return f'\n\tEphemeris data for {len(EPH)} satellites\n' + tabulate(EPH, headers=self.EPH[0]) + '\n\n' + \
        #     f'\tAlmanac data for {len(ALM)} satellites\n' + tabulate(ALM, headers=self.ALM[0])

def calc_sat_alm(ALM: list, t):
    # SV_ID, week, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health, Data_ID, receiving_time = ALM
    SV_ID = ALM[0] # ID спутника
    week = ALM[1] # номер недели передаваемых данных
    Toa = ALM[2] # опорное время внутри недели N, на которую передаются данные альманах
    e = ALM[3] # эксцентриситет
    di = ALM[4] * pi # rad, поправка к наклонению
    Wdot = ALM[5] * pi # rad/s, скорость прецессии орбиты
    sqrtA = ALM[6] # корень из большей полуоси
    W0 = ALM[7] * pi # rad Угол восходящего узла на момент начала недели N
    w = ALM[8] * pi # rad аргумент перигея
    M0 = ALM[9] * pi # rad средняя аномалия на эпоху Toa
    af0 = ALM[10] #
    af1 = ALM[11] #
    health = ALM[12] #
    Data_ID = ALM[13] #
    receiving_time: datetime = ALM[14] # время принятия сигнала

    mu = 3.986005 * 1e14 # m^3/s^2 гравитационная постоянная для земли WGS-84
    Wedot = 7.2921151467 * 10e-5 # rad/s скорость вращения земли WGS-84
    i0 = 0.30 * pi # rad

    A = sqrtA ** 2  # большая полуось
    n0 = sqrt(mu / A ** 3)  # rad/s вычисленное среднее перемещение
    # tk = t - Toa  # время от эпохи отсчета эфимерид
    # if tk > 302400:
    #     tk -= 604800
    # elif tk < 302400:
    #     tk += 604800

    tk=0

    n = n0   # скорректированное средне движение
    Mk = M0 + n * tk  # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    E0 = Mk  # rad
    for i in range(10):
        E1 = E0 + (Mk - E0 + e * sin(E0)) / (1 - e * cos(E0))
        E0 = E1
    E = E1  # rad
    v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(E / 2))  # истиная аномалия
    u = v + w  # аргумент lat
    r = A * (1 - e * cos(E))   # скорректированный радиус
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

    mu = 3.986005 * 1e14 # m^3/s^2 гравитационная постоянная для земли WGS-84
    Wedot = 7.2921151467 * 10e-5 # rad/s скорость вращения земли WGS-84

    # dTsv = af0 + af
    # t = Toe

    A = sqrtA**2 # большая полуось
    n0 = sqrt( mu / A**3 ) # rad/s вычисленное среднее перемещение
    tk = t - Toe # время от эпохи отсчета эфимерид
    if tk > 302400:
        tk -= 604800
    elif tk < 302400:
        tk += 604800
    n = n0 + dn # скорректированное средне движение
    Mk = M0 + n*tk # средняя аномалия
    ## Решение уравнения Mk = Ek - e * sin(Ek)
    E0 = Mk # rad
    for i in range(10):
        E1 = E0 + (Mk-E0 + e*sin(E0)) / (1 - e*cos(E0))
        E0 = E1
    E = E1 # rad
    # v = 2 * math.atan(sqrt((1+e)/(1-e)) * tan(E/2)) # истиная аномалия
    v = atan2(
        sqrt(1 - e * e) * sin(E),
        cos(E) - e)
    Phi = v + w # аргумент lat
    du = Cus*sin(2*Phi) + Cuc*cos(2*Phi) # коррекция lat
    dr = Crs*sin(2*Phi) + Crc*cos(2*Phi) # коррекция радиуса
    di = Cis*sin(2*Phi) + Cic*cos(2*Phi) # коррекция наклона
    u = Phi + du # скорректированный агрумент lat
    r = A*(1-e*cos(E)) + dr # скорректированный радиус
    i = i0 + di + IDOT*tk # скорректированный наклон
    x1 = r*cos(u) # позиция в плоскости орбиты
    y1 = r*sin(u) # позиция в плоскости орбиты
    W = W0 + (Wdot-Wedot)*tk - Wedot*Toe # исправленная долгота lon восходящего узла
    x = x1*cos(W) - y1*cos(i)*sin(W)
    y = x1*sin(W) + y1*cos(i)*cos(W)
    z = y1*sin(i)

    return u, W, i, Mk, r, x, y, z
    # u - скорректированный агрумент lat
    # i - скорректированный наклон
    # W - исправленная долгота lon восходящего узла
