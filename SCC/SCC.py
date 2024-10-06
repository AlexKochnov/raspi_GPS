from GNSS import GNSS, NavDataType
from GPS_CC import calc_gps_alm, calc_gps_eph
from GLONASS_CC import calc_glo_alm, calc_glo_eph_simple
from TimeStamp import TimeStamp


class SatellitesCoordinateCalculator:
    def __init__(self, parameters, gnssId: GNSS, mode: NavDataType):
        # self.parameters = parameters
        self.next_func = self.choose_func(parameters, gnssId, mode)

    def choose_func(self, parameters, gnssId, mode):
        match (gnssId, mode):
            case (GNSS.GPS, NavDataType.ALM):
                return lambda rcvTow, week: calc_gps_alm(parameters, rcvTow, week)
            case (GNSS.GPS, NavDataType.EPH):
                return lambda rcvTow, week: calc_gps_eph(parameters, rcvTow, week)
            case (GNSS.GLONASS, NavDataType.ALM):
                return lambda rcvTow, week: calc_glo_alm(parameters, *TimeStamp.gps2glonass(week, rcvTow))
            case (GNSS.GLONASS, NavDataType.EPH):
                return lambda rcvTow, week: calc_glo_eph_simple(parameters, *TimeStamp.gps2glonass(week, rcvTow))
                # self.runer = RK45Solver(*get_glo_eph_simple_func(data))
                # return lambda stamp: self.runer.get(stamp.to_glonass()[-1])

    def get(self, rcvTow, week):
        return self.next_func(rcvTow, week)