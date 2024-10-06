from Filters import FederatedKalmanFilter
from GNSS import GNSS, GNSSLen, Source, NavDataType
from Satellite import Satellite
from TimeStamp import TimeStamp


class DataStore:
    satellites: list[Satellite]
    parameters: dict
    filter_xyz: FederatedKalmanFilter
    filter_lla: FederatedKalmanFilter
    stamp: TimeStamp = None
    sources: list[Source]

    def __init__(self, gnssId_s=None):
        if gnssId_s is None:
            gnssId_s = [GNSS.GPS]

        self.satellites = []
        for gnssId in gnssId_s:
            for svId in range(1, 1 + GNSSLen[gnssId]):
                self.satellites.append(Satellite(gnssId, svId))

        self.sources = Source.multi_get(gnssId_s)
        self.filter_xyz = FederatedKalmanFilter(self.sources)
        self.filter_lla = FederatedKalmanFilter(self.sources)


    def update(self, message):
        pass