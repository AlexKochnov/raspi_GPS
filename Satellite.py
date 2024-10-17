import typing

import numpy as np

import Constants
from GNSS import GNSS, NavDataType
from TimeStamp import TimeStamp

from SCC import SatellitesCoordinateCalculator as SCC

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

class SCC_Block:
    scc: SCC
    gnssId: GNSS
    mode: NavDataType

    def __init__(self, gnssId: GNSS, mode: NavDataType):
        self.gnssId = gnssId
        self.mode = mode
        self.scc = SCC(gnssId, mode)

    def update_parameters(self, parameters: dict):
        self.scc.update(parameters)

    def get(self, rcvTow, week):
        return self.scc.get(rcvTow, week)

    def check_parameters_valid(self):
        if self.scc:
            return True
        return False

    def check_glonass(self):
        if self.scc.parameters.get('ln', 1) == 0:
            #TODO: добавить проверку частично обновленных данных
            sfNs = [self.scc.parameters.get(f'sfN{i}', None) for i in range(1, 5)]
            if not None in sfNs:
                mx = max(sfNs)
                mi = min(sfNs)
                if mx-mi <= 1: # данные постепенно приходят
                    return True
        return False

class Satellite:
    gnssId: GNSS
    svId: int
    ephemeris: SCC_Block = None
    almanac: SCC_Block = None
    history_prMes: list[(int, float)]
    prMes: float or object = None
    rcvTow: float = None
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

    # storage = None

    def __init__(self, gnssId: GNSS, svId: int):#, storage_data: dict):
        self.gnssId = gnssId
        self.svId = svId
        self.history_prMes = []
        self.almanac = SCC_Block(gnssId, NavDataType.ALM)
        self.ephemeris = SCC_Block(gnssId, NavDataType.EPH)
        # self.storage = storage_data

    def __save_update__(self, data):
        type_hints = typing.get_type_hints(self.__class__)
        for key, value in data.items():
            if hasattr(self, key):
                expected_type = type_hints[key]
                value = expected_type(value)
                setattr(self, key, value)

    def calc_position(self, rcvTow, week, dataType: NavDataType):
        if dataType == NavDataType.ALM:
            return self.almanac.get(rcvTow, week)
        else:
            return self.ephemeris.get(rcvTow, week)

    def get_calculation_dict(self, rcvTow, week, dataType: NavDataType):
        af_dt, xyz = self.calc_position(rcvTow, week, dataType)
        if xyz is None or self.prMes is None:
            return None
        return {
            'X': xyz[0],
            'Y': xyz[1],
            'Z': xyz[2],
            'af_dt': af_dt,
            'prMes': self.prMes,
            'nav_score': self.calc_nav_score(),
            'svId': self.svId,
            'gnssId': self.gnssId,
        }


    def update_nav(self, data: dict, time_stamp: TimeStamp):
        self.nav_stamp = time_stamp
        self.__save_update__(data)

    def update_alm(self, data: dict):
        if data is not None:
            self.almanac.update_parameters(data)

    def update_eph(self, data: dict):
        if data is not None:
            self.ephemeris.update_parameters(data)

    def update_prmes(self, data: dict, rcvTow: float):
        # self.prMes = data['prMes']
        self.rcvTow = rcvTow
        self.history_prMes.append((rcvTow, data['prMes']))
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
        # если общая информация устарела не больше, чем на 10 сек
        # nav_young = abs((self.rcvTow - self.nav_stamp.TOW) % Constants.week_seconds) < 10.5  if self.rcvTow else False
        if self.health and self.prValid == True:
            return S_quality * S_cno * S_prRes * S_prRMSer * S_prStedv * S_visibility
        return 0

    def calc_eph_score(self):
        return self.ephAvail and self.almanac.check_parameters_valid()

    def calc_alm_score(self):
        return self.almAvail and self.ephemeris.check_parameters_valid()

