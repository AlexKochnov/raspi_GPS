import typing

import numpy as np

from GNSS import GNSS
from TimeStamp import TimeStamp

def get_scores(data, *args, BiggerBetter=True) -> int:
    score = 0
    if BiggerBetter:
        for arg in sorted(args):
            if data >= arg:
                score += 1
    else:
        for arg in sorted(args)[::-1]:
            if data <= arg:
                score += 1
    return score

class Satellite:
    gnssId: GNSS
    svId: int
    ephemeris: object = None
    almanac: object = None
    history_prMes: list[(int, float)]
    prMes: float or object
    rcvTow: float
    nav_stamp: TimeStamp = None
    prRes: float = np.nan
    prRMSer: float = np.nan
    prStedv: float = np.nan
    elev: float = -91
    azim: float = 0
    cno: int = 0
    qualityInd: int = 1
    mpathIndic: int = 4
    visibility: int = 0
    ephAvail: bool = False
    almAvail: bool = False
    health: bool = False
    svUsed: bool = False
    prValid: bool = False

    def __init__(self, gnssId: GNSS, svId: int, storage_data: dict):
        self.gnssId = gnssId
        self.svId = svId
        self.history_prMes = []

    def __save_update__(self, data):
        type_hints = typing.get_type_hints(self.__class__)
        for key, value in data.items():
            if hasattr(self, key):
                expected_type = type_hints[key]
                value = expected_type(value)
                setattr(self, key, value)

    def update_nav(self, data: dict, time_stamp: TimeStamp):
        self.nav_stamp = time_stamp
        self.__save_update__(data)

    def update_alm(self, data: dict):
        pass

    def update_eph(self, data: dict):
        pass

    def update_prmes(self, data: dict):
        # self.prMes = data['prMes']
        # self.rcvTow = data['rcvTow']
        self.history_prMes.append((data['rcvTow'], data['prMes']))
        self.__save_update__(data)

    def get_score(self) -> (int, int, int):
        N, A, E = self.calc_nav_score(), self.calc_alm_score(), self.calc_eph_score()
        return N, A, E

    def calc_nav_score(self):
        S_quality = get_scores(self.qualityInd,4, 5)
        S_cno = get_scores(self.cno, 20, 30)
        S_prRes = get_scores(abs(self.prRes), 10, 40, BiggerBetter=False)  # + 1
        S_prRMSer = get_scores(self.prRMSer, 10, 40, BiggerBetter=False)
        S_prStedv = get_scores(self.prStedv, 10, 40, BiggerBetter=False)
        S_visibility = get_scores(self.visibility, 2, 3)
        if self.health and self.prValid == True:
            return S_quality * S_cno * S_prRes * S_prRMSer * S_prStedv * S_visibility
        return 0

    def calc_eph_score(self):
        return self.ephAvail

    def calc_alm_score(self):
        return self.almAvail

