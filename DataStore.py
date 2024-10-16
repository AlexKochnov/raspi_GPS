import NMEAMessages
import Settings
import UBXMessages
from Filters import FederatedKalmanFilter
from GNSS import GNSS, GNSSLen, Source, NavDataType, ReceiverSource
from Satellite import Satellite
from TimeStamp import TimeStamp
from Transformations import ecef2lla

from solve_nav_task import solve_nav_task
import serialize

class DataStore:
    satellites: dict[(GNSS, int), Satellite]
    parameters: dict
    filter_xyz: FederatedKalmanFilter
    filter_lla: FederatedKalmanFilter
    stamp: TimeStamp = None
    sources: list[Source]
    used_gnss: list[GNSS]
    multi_gnss_task: bool = False
    queue: dict = None

    def __init__(self, *gnssId_s):
        if not gnssId_s:
            gnssId_s = [GNSS.GPS]
        self.used_gnss = gnssId_s

        self.satellites = {}
        for gnssId in gnssId_s:
            for svId in range(1, 1 + GNSSLen[gnssId]):
                self.satellites[(gnssId, svId)] = Satellite(gnssId, svId)

        self.parameters = {}

        self.sources = Source.multi_get(gnssId_s, self.multi_gnss_task)
        self.filter_xyz = FederatedKalmanFilter(self.sources)
        self.filter_lla = FederatedKalmanFilter(self.sources)

        self.queue = dict()

    def check_satellite(self, gnssId: GNSS, svId: int) -> bool:
        """
        Проверка наличия спутника среди хранимых
        :param gnssId: GNSS - Идентификатор ГНСС спутника
        :param svId: int - Номер спутника
        :return: bool - Флаг наличия спутника
        """
        return (gnssId, svId) in self.satellites

    def update(self, message: UBXMessages or NMEAMessages):
        """
        Обновление данных на основе информации из нового сообщения
        :param message: UBXMessages or NMEAMessages - Новое сообщение
        :return:
        """
        if not isinstance(message, UBXMessages.UbxMessage or NMEAMessages.NmeaMessage):
            return

        self.time_correction(message)

        if isinstance(message, UBXMessages.UbxMessage):
            self.update_ubx(message)
        elif isinstance(message, NMEAMessages.NmeaMessage):
            pass

        self.check_tick(message)

    def time_correction(self, message: UBXMessages or NMEAMessages):
        """
        Коррекция времени нового сообщения или хранимой последней метки в хранилище
        :param message: UBXMessages or NMEAMessages - Новое сообщение
        :return:
        """
        if message.data is not None:
            if 'iTOW' in message.data:
                message.receiving_stamp.TOW = message.data['iTOW'] / 1000
            if 'week' in message.data:
                self.parameters['week'] = message.data['week']
                message.receiving_stamp.week = message.data['week']
        if 'week' in self.parameters:
            message.receiving_stamp.week = self.parameters['week']
        self.stamp = message.receiving_stamp

    def check_tick(self, message: UBXMessages or NMEAMessages):
        """
        Проверка конца прихода оперативной информации для расчета
        :param message: UBXMessages or NMEAMessages - Новое сообщение
        :return:
        """
        self.queue[type(message)] = message.receiving_stamp.TOW
        if {UBXMessages.RXM_RAWX, UBXMessages.NAV_POSECEF} <= set(self.queue.keys()):
            self.tick()
            self.queue.clear()

    def tick(self):
        """
        Обработка данных за текущую секунду
        :return:
        """
        for gnssId in self.used_gnss:
            self.solve_nav_task_for_satellite_group(gnssId)
        if self.multi_gnss_task:
            self.solve_nav_task_for_satellite_group(GNSS.ALL)
        self.filter_xyz.choose_next()
        self.filter_lla.choose_next()

    def solve_nav_task_for_satellite_group(self, chosen_gnssId: GNSS):
        """
        Решение навигационной задачи для заданной системы ГНСС
        :param chosen_gnssId: GNSS - Идентификатор систем GPS
        :return:
        """
        ## Первая чистка: по GNSS ID
        if chosen_gnssId == GNSS.ALL:
            chosen = self.satellites.items()
        else:
            chosen = [satellite for (gnssId, svId), satellite in self.satellites.items() if gnssId == chosen_gnssId]

        ## Вторая чистка:
        alm_sats = []
        eph_sats = []
        for sat in chosen:
            N, A, E = sat.get_score()
            if N > Settings.MinimumNavigationSatelliteScore:
                if A:
                    alm_sats.append(sat)
                if E:
                    eph_sats.append(sat)

        rcvTow = self.parameters.get('rcvTow', None)
        week = self.parameters.get('week', None)
        if rcvTow is None or week is None:
            return
        assert isinstance(rcvTow,float) and isinstance(week, int)

        for sats, type in zip([alm_sats, eph_sats], [NavDataType.ALM, NavDataType.EPH]):
            solve = solve_nav_task(sats, rcvTow, week, type)
            if solve['success']:
                xyz = solve['result'][:3]
                lla = ecef2lla(*list(xyz))
                ts = TimeStamp(TOW=rcvTow, week=week)
                self.filter_xyz.update(Source(chosen_gnssId, type), xyz, solve, ts)
                self.filter_lla.update(Source(chosen_gnssId, type), lla, solve, ts)

    def update_ubx(self, message: UBXMessages):
        """
        Обновление данных при условии, что новое сообщение формата UBX
        :param message: UBXMessages - Новое сообщение
        :return:
        """
        if isinstance(message, UBXMessages.AID_ALM):
            self.satellites[(message.stamp['gnssId'], message.stamp['svId'])].update_alm(message.data)
        if isinstance(message, UBXMessages.AID_EPH):
            self.satellites[(message.stamp['gnssId'], message.stamp['svId'])].update_eph(message.data)
        if isinstance(message, UBXMessages.NAV_SAT | UBXMessages.NAV_ORB | UBXMessages.RXM_MEASX | UBXMessages.RXM_SVSI):
            for sat in message.satellites:
                sat_stamp = (sat['gnssId'], sat['svId'])
                if self.check_satellite(*sat_stamp):
                    self.satellites[sat_stamp].update_nav(sat, message.receiving_stamp)
        if isinstance(message, UBXMessages.NAV_POSECEF | UBXMessages.NAV_VELECEF | UBXMessages.NAV_DOP | \
                               UBXMessages.NAV_CLOCK | UBXMessages.NAV_TIMEGPS):
            self.parameters.update(message.data)
            if isinstance(message, UBXMessages.NAV_POSECEF):
                self.queue['POSECEF'] = message.data['iTOW'] / 1000
                xyz = [message.data[key] for key in [f'ecef{sym}' for sym in 'XYZ']]
                lla = ecef2lla(*xyz)
                solve = {
                    'GDOP': self.parameters.get('gDOP', 100),
                    'success': True
                }
                self.filter_xyz.update(ReceiverSource, xyz, solve, self.stamp)
                self.filter_lla.update(ReceiverSource, lla, solve, self.stamp)
        if isinstance(message, UBXMessages.RXM_SFRBX):
            if message.data['gnssId'] == GNSS.GPS:
                pass
            elif message.data['gnssId'] == GNSS.GLONASS:
                if message.data['id'] == -1:
                    if 'NA' in message.signal.keys():
                        self.parameters['NA'] = message.signal['NA']
                    self.parameters.update(message.signal)
                elif message.data['id'] == 0: # эфемериды -> svId = data['svId']
                    sat_stamp = (GNSS.GLONASS, message.data['svId'])
                    if self.check_satellite(*sat_stamp):
                        self.satellites[sat_stamp].update_eph(message.signal)
                else:
                    sat_stamp = (GNSS.GLONASS, message.data['id'])
                    if self.check_satellite(*sat_stamp):
                        self.satellites[sat_stamp].update_alm(message.signal | {'NA': self.parameters['NA']})
        if isinstance(message, UBXMessages.RXM_RAWX):
            self.queue['RAWX'] = round(message.data['rcvTow'])
            for sat in message.satellites:
                sat_stamp = (sat['gnssId'], sat['svId'])
                if self.check_satellite(*sat_stamp):
                    self.satellites[sat_stamp].update_prmes(sat, message.data['rcvTow'])
            self.parameters['rcvTow'] = message.data['rcvTow']

    def serialize(self):
        """
        Сериализация и сохранение расчетов
        :return:
        """
        folder = 'serialized_data\\'
        for i in range(2):
            format = 'xyz' if i == 0 else 'lla'
            fkf_filter = self.filter_xyz if i == 0 else self.filter_lla
            serialize.serialize_FKF(fkf_filter).to_csv(f'{folder}fkf_{format}_{Settings.START_ID}.csv')
            for source, lkf in fkf_filter.filters.items():
                serialize.serialize_LKF(lkf).\
                    to_csv(f'{folder}lkf_{format}_{source.gnssId.name}_{source.dataType.name}_{Settings.START_ID}.csv')
                a=0

