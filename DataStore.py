import Settings
import UBXMessages
from Filters import FederatedKalmanFilter
from GNSS import GNSS, GNSSLen, Source, NavDataType, ReceiverSource
from Satellite import Satellite
from TimeStamp import TimeStamp
from Transformations import ecef2lla

from solve_nav_task import solve_nav_task


class DataStore:
    satellites: dict[(GNSS, int), Satellite]
    parameters: dict
    filter_xyz: FederatedKalmanFilter
    filter_lla: FederatedKalmanFilter
    stamp: TimeStamp = None
    sources: list[Source]
    used_gnss: list[GNSS]
    multi_gnss_task: bool = False

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


    def check_satellite(self, gnssId, svId):
        return (gnssId, svId) in self.satellites

    def update(self, message):
        if isinstance(message, UBXMessages.UbxMessage):
            self.update_ubx(message)
        pass

    def tick(self):
        for gnssId in self.used_gnss:
            self.solve_nav_task_for_satellite_group(gnssId)
        if self.multi_gnss_task:
            self.solve_nav_task_for_satellite_group(GNSS.ALL)

        self.filter_xyz.choose_next()
        self.filter_lla.choose_next()

    def solve_nav_task_for_satellite_group(self, chosen_gnssId: GNSS):
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
                self.filter_xyz.update(Source(chosen_gnssId, type),lla, solve, ts)
                self.filter_lla.update(Source(chosen_gnssId, type),lla, solve, ts)


    def update_ubx(self, message):
        if message.data is not None:
            if 'iTOW' in message.data:
                message.receiving_stamp.TOW = message.data['iTOW'] / 1000
            if 'week' in message.data:
                self.parameters['week'] = message.data['week']
                message.receiving_stamp.week = message.data['week']

        self.stamp = message.receiving_stamp

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
                xyz = [message.data[key] for key in [f'ecef{sym}' for sym in 'XYZ']]
                lla = ecef2lla(*xyz)
                solve = {
                    'GDOP': self.parameters.get('GDOP', 100),
                    'success': True
                }
                self.filter_xyz.update(ReceiverSource, xyz, solve, self.stamp)
                self.filter_lla.update(ReceiverSource, lla, solve, self.stamp)

        if isinstance(message, UBXMessages.RXM_SFRBX):
            pass
        if isinstance(message, UBXMessages.RXM_RAWX):
            for sat in message.satellites:
                sat_stamp = (sat['gnssId'], sat['svId'])
                if self.check_satellite(*sat_stamp):
                    self.satellites[sat_stamp].update_prmes(sat, message.data['rcvTow'])
            self.parameters['rcvTow'] = message.data['rcvTow']

            self.tick()


