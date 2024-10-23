import typing
import numpy as np

from Utils.GNSS import GNSS, NavDataType
from Utils.TimeStamp import TimeStamp
from SCC import SatellitesCoordinateCalculator as SCC

def get_scores(data: float or int, *args: float or int, BiggerBetter=True) -> int:
    """
    Функция приведения значения из диапазона к дискретному множеству
    :param data: float or int - оцениваемый параметр
    :param args: float or int - диапазоны для оценки
    :param BiggerBetter: bool - флаг того, какое значение лучше - большее или меньшее (точка отсчета)
    :return:
    """
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
    """
    Класс для управления расчетом координат и хранения необходимых параметров
    """
    scc: SCC
    gnssId: GNSS
    mode: NavDataType

    def __init__(self, gnssId: GNSS, mode: NavDataType):
        self.gnssId = gnssId
        self.mode = mode
        self.scc = SCC(gnssId, mode)

    def update_parameters(self, parameters: dict):
        """
        Обновление параметров для расчета
        :param parameters: dict - данные для обновления
        :return:
        """
        self.scc.update(parameters)

    def get(self, rcvTow: float, week: int) -> (float, np.array or None):
        """
        Получение рассчитанных координат
        :param rcvTow: float - время расчета
        :param week: int -неделя расчета
        :return: (float, np.array or None) - задержка спутника и его вектор состояния
        """
        return self.scc.get(rcvTow, week)

    def check_parameters_valid(self):
        """
        Проверка качества текущих параметров
        :return: bool - флаг качества
        """
        if self.scc:
            return True
        return False

    def check_glonass(self):
        """
        Проверка качества параметров для спутника ГЛОНАСС
        :return: bool - флаг качества
        """
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
    """
    Класс для хранения и обработки всей информации о конкретном спутнике
    """
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

    def __init__(self, gnssId: GNSS, svId: int):#, storage_data: dict):
        self.gnssId = gnssId
        self.svId = svId
        self.history_prMes = []
        self.almanac = SCC_Block(gnssId, NavDataType.ALM)
        self.ephemeris = SCC_Block(gnssId, NavDataType.EPH)

    def __save_update__(self, data: dict):
        """
        Обновление текущих атрибутов из заданного словаря
        :param data: dict
        :return:
        """
        type_hints = typing.get_type_hints(self.__class__)
        for key, value in data.items():
            if hasattr(self, key):
                expected_type = type_hints[key]
                value = expected_type(value)
                setattr(self, key, value)

    def calc_position(self, rcvTow: float, week: int, dataType: NavDataType) -> (float, np.array or None):
        """
        Функция получения позиции спутника в заданное время по заданному типу данных
        :param rcvTow: float - время расчета от начала недели (время получения псевдодальностей)
        :param week: int - неделя расчета
        :param dataType: NavDataType - тип данных для расчета
        :return: (float, np.array or None) - задержка спутника и его вектор состояния
        """
        if dataType == NavDataType.ALM:
            return self.almanac.get(rcvTow, week)
        else:
            return self.ephemeris.get(rcvTow, week)

    def get_calculation_dict(self, rcvTow: float, week: int, dataType: NavDataType) -> None or dict:
        """
        Получение словаря с данными, необходимыми для решения навигационной задачи
        :param rcvTow: float - время расчета
        :param week: int - неделя расчета
        :param dataType: NavDataType - тип данных для расчета
        :return: dict or None - словарь данных для расчета
        """
        af_dt, xyz = self.calc_position(rcvTow, week, dataType)
        if xyz is not None and not all(np.isnan(xyz)):
            with open('satellite_table.csv', 'a') as file:
                file.write(f'{week};{int(rcvTow)};{self.svId};{self.gnssId.value};{xyz[0]};{xyz[1]};{xyz[2]};{'a' if dataType == NavDataType.ALM else 'e'}\n')

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
        """
        Обновить общие навигационные данные
        :param data: dict - данные для обновления
        :param time_stamp: TimeStamp - метка времени обновления
        :return:
        """
        self.nav_stamp = time_stamp
        self.__save_update__(data)

    def update_alm(self, data: dict):
        """
        Обновление данных альманах
        :param data: dict - данные для обновления
        :return:
        """
        if data is not None:
            self.almanac.update_parameters(data)

    def update_eph(self, data: dict):
        """
        Обновление данных эфемерид
        :param data: dict - данные для обновления
        :return:
        """
        if data is not None:
            self.ephemeris.update_parameters(data)

    def update_rawx(self, data: dict, rcvTow: float):
        """
        Обновить данные об измерениях
        :param data: dict - данные для обновления
        :param rcvTow: float - время измерения
        :return:
        """
        # self.prMes = data['prMes']
        self.rcvTow = rcvTow
        self.history_prMes.append((rcvTow, data['prMes']))
        self.__save_update__(data)

    def get_score(self) -> (int, int, int):
        """
        Рассчитать оценку качества спутника
        :return: (int, int, int) - очки по общим навигационным данным, данным альманах и эфемерид
        """
        N, A, E = self.calc_nav_score(), self.calc_alm_score(), self.calc_eph_score()
        return N, A, E

    def calc_nav_score(self) -> int:
        """
        Оценка качества навигационных данных
        :return: int - очки навигации
        """
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

    def calc_eph_score(self) -> int:
        """
        Оценка качества данных эфемерид
        :return: int - очки эфемерид
        """
        return int(self.ephAvail and self.almanac.check_parameters_valid())

    def calc_alm_score(self) -> int:
        """
        Оценка качества данных альманах
        :return: int - очки альманах
        """
        return int(self.almAvail and self.ephemeris.check_parameters_valid())

