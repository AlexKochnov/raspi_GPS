import math
from datetime import datetime, timedelta
from typing import Union

from Utils import Constants
from Utils.Constants import gps_epoch, leapS, week_seconds, tz_utc, STEP
from Utils.GNSS import GNSS


class TimeStamp:
    dt: datetime
    TOW: float
    week: int

    def __init__(self, *args: Union[float, int, datetime], **kwargs: Union[float, int, datetime]):
        """
        Инициализирует объект TimeStamp.

        Может быть вызван без аргументов, с двумя позиционными аргументами
        (TOW и неделя) или с двумя именованными аргументами (TOW и неделя).

        :param args: Позиционные аргументы (TOW и неделя).
        :param kwargs: Именованные аргументы (TOW и неделя).
        :param type: тип передаваемого времени
        """
        type = kwargs.get('type', GNSS.GPS)
        if not args and not kwargs:
            self.dt = datetime.now(tz=tz_utc)
            self.TOW, self.week = self.dt2gps(self.dt)
        elif len(args) == 2 and all(isinstance(arg, (int, float)) for arg in args):
            if type == GNSS.GPS:
                self.TOW, self.week = float(args[0]), int(args[1])
            elif type == GNSS.GLONASS:
                pass
        elif len(kwargs) == 2 and all(key in kwargs for key in ['TOW', 'week']):
            self.TOW, self.week = float(kwargs['TOW']), int(kwargs['week'])
        elif len(args) == 1 and isinstance(args[0], datetime):
            self.from_datetime(args[0])
        elif len(kwargs) == 1 and 'datetime' in kwargs:
            # assert isinstance(kwargs['datetime'], datetime)
            self.from_datetime(kwargs['datetime'])

    def from_datetime(self, dt: datetime):
        self.TOW, self.week = self.dt2gps(dt)
        pass
    
    def total_stamp(self):
        return self.TOW + self.week * Constants.week_seconds

    def to_datetime(self) -> datetime:
        return gps_epoch + timedelta(seconds=self.total_stamp() - Constants.leapS)

    @staticmethod
    def gps2glonass(week, tow):
        # dt_gps = gps_epoch + timedelta(seconds=tow - Constants.leapS, days=week*7)
        dt_gps = TimeStamp.gps2dt(tow, week)
        N4, N, t = TimeStamp.dt2glonass(dt_gps)
        return t, N, N4

    # @staticmethod
    # def gps2dt(week, TOW):
    #     return Constants.gps_epoch + timedelta(seconds=TOW, weeks=week)

    @staticmethod
    def dt2glonass(dt):
        delta = dt - Constants.glonass_epoch + timedelta(hours=3)
        N4 = delta.days // Constants.glonass_4years.days + 1
        N = (delta.days % Constants.glonass_4years.days) + 1
        t = (delta - timedelta(days=delta.days)).total_seconds()
        t = TimeStamp.round_step(t, STEP)
        return N4, N, t

    @staticmethod
    def glo2dt(t1, N, N4):
        dt_gl = Constants.glonass_epoch + timedelta(seconds=t1, days=N - 1, hours=-3) + (N4 - 1) * Constants.glonass_4years
        return dt_gl

    @staticmethod
    def gps2dt(tow, week):
        dt_gps = Constants.gps_epoch + timedelta(seconds=tow - Constants.leapS, days=7 * week)
        return dt_gps

    def to_glonass(self):
        return self.dt2glonass(self.dt)

    def to_gps(self):
        return self.week, self.TOW

    def __str__(self):
        return f'_|TS: {self.week}.{self.TOW}|_'

    def __repr__(self):
        return str(self)

    @staticmethod
    def round_step(data, step=STEP):
        return round(data / step) * step

    @staticmethod
    def dt2gps(timestamp: datetime):
        dt = (timestamp - gps_epoch).total_seconds() + leapS
        TOW = dt % week_seconds
        week = int(dt // week_seconds)
        TOW = TimeStamp.round_step(TOW)
        return TOW, week

    def __add__(self, other: Union[int, float]):
        """
        Сложение объекта TimeStamp с числом (секунды).
        """
        if not isinstance(other, (int, float)):
            return NotImplemented

        new_tow = self.TOW + other
        new_week = self.week
        if new_tow >= Constants.week_seconds:
            new_week += int(new_tow // Constants.week_seconds)
            new_tow = new_tow % Constants.week_seconds

        return TimeStamp(new_tow, new_week)

    def __sub__(self, other: Union['TimeStamp', int, float]):
        """
        Вычитание объекта TimeStamp из другого TimeStamp или числа (секунды).
        """
        if isinstance(other, TimeStamp):
            return self.total_stamp() - other.total_stamp()
        elif isinstance(other, (int, float)):
            new_tow = self.TOW - other
            new_week = self.week
            if new_tow < 0:
                new_week += int(new_tow // Constants.week_seconds)
                new_tow = new_tow % Constants.week_seconds

            return TimeStamp(new_tow, new_week)
        else:
            return NotImplemented

    def __float__(self):
        return self.total_stamp()

    # Определение операций сравнения
    def __lt__(self, other):
        if isinstance(other, (int, float)):
            return self.total_stamp() < float(other)
        elif isinstance(other, TimeStamp):
            return self.total_stamp() < other.total_stamp()
        else:
            return NotImplemented

    def __le__(self, other):
        if isinstance(other, (int, float)):
            return self.total_stamp() <= float(other)
        elif isinstance(other, TimeStamp):
            return self.total_stamp() <= other.total_stamp()
        else:
            return NotImplemented

    def __eq__(self, other):
        if isinstance(other, (int, float)):
            return self.total_stamp() == float(other)
        elif isinstance(other, TimeStamp):
            return self.total_stamp() == other.total_stamp()
        else:
            return NotImplemented


# def get_now_TOW():
#     return TimeStamp.DT2TOW(datetime.now(tz=tz_utc), STEP)
# BASE_TIME_STAMP = get_now_TOW

BASE_TIME_STAMP = TimeStamp


def calc_julian_day(N4: int, N: int) -> tuple[int, int, int, int, float]:
    JD0 = 1461 * (N4 - 1) + N + 2450082.5
    JDN = JD0 + 0.5
    a = JDN + 32044
    b = (4 * a + 3) // 146097
    c = a - (146097 * b) // 4
    d = (4 * c + 3) // 1461
    e = c - (1461 * d) // 4
    m = (5 * e + 2) // 153

    day: int = e - (153 * m + 2) // 5 + 1
    month: int = m + 3 - 12 * (m // 10)
    year: int = 100 * b + d - 4800 + (m // 10)
    day_of_week: int = JDN % 7 + 1

    ERA = 2 * math.pi * (0.7790572732640 + 1.00273781191135448 * (JD0 - 2451545.0))
    dT = (JD0 - 2451545.0) / 36525
    GMST = ERA + 0.0000000703270726 + 0.0223603658710194 * dT + 0.0000067465784654 * dT ** 2 - 0.0000000000021332 * dT ** 3 - 0.0000000001452308 * dT ** 4 - 0.0000000000001784 * dT ** 5
    return day, month, year, day_of_week, GMST