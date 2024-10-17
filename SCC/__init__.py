import numpy as np

from Utils.GNSS import GNSS, NavDataType
from .GPS_CC import calc_gps_alm, calc_gps_eph
from .GLONASS_CC import calc_glo_alm, calc_glo_eph_simple
from Utils.TimeStamp import TimeStamp


class SatellitesCoordinateCalculator:
    def __init__(self, gnssId: GNSS, mode: NavDataType, parameters = None):
        self.parameters = parameters or {}
        self.next_func = self.choose_func(gnssId, mode)

    def choose_func(self, gnssId, mode):
        match (gnssId, mode):
            case (GNSS.GPS, NavDataType.ALM):
                return lambda rcvTow, week: calc_gps_alm(self.parameters, rcvTow, week)
            case (GNSS.GPS, NavDataType.EPH):
                return lambda rcvTow, week: calc_gps_eph(self.parameters, rcvTow, week)
            case (GNSS.GLONASS, NavDataType.ALM):
                return lambda rcvTow, week: calc_glo_alm(self.parameters, *TimeStamp.gps2glonass(week, rcvTow))
            case (GNSS.GLONASS, NavDataType.EPH):
                return lambda rcvTow, week: calc_glo_eph_simple(self.parameters, *TimeStamp.gps2glonass(week, rcvTow))
                # self.runer = RK45Solver(*get_glo_eph_simple_func(data))
                # return lambda stamp: self.runer.get(stamp.to_glonass()[-1])

    def get(self, rcvTow, week):
        try:
            af_dt, state = self.next_func(rcvTow, week)
        except:
            af_dt, state = 0, np.array([np.nan] * 3)
        return af_dt, state

    def update(self, parameters):
        self.parameters.update(parameters)