import math
from datetime import datetime, timedelta
from typing import Union

import Constants
from Constants import gps_epoch, leapS, week_seconds, tz_utc, STEP


class TimeStamp:
    TOW: float
    week: int

    def __init__(self, *args: Union[float, int], **kwargs: Union[float, int]):
        """
        Инициализирует объект TimeStamp.

        Может быть вызван без аргументов, с двумя позиционными аргументами
        (TOW и неделя) или с двумя именованными аргументами (TOW и неделя).

        :param args: Позиционные аргументы (TOW и неделя).
        :param kwargs: Именованные аргументы (TOW и неделя).
        """
        if not args and not kwargs:
            self.TOW, self.week = self.DT2TOW(datetime.now(tz=tz_utc), STEP)
        elif len(args) == 2 and all(isinstance(arg, (int, float)) for arg in args):
            self.TOW, self.week = float(args[0]), int(args[1])
        elif len(kwargs) == 2 and all(key in kwargs for key in ['TOW', 'week']):
            self.TOW, self.week = float(kwargs['TOW']), int(kwargs['week'])

    def total_stamp(self):
        return self.TOW + self.week * Constants.week_seconds

    def to_datetime(self) -> datetime:
        return gps_epoch + timedelta(seconds=self.total_stamp() - Constants.leapS)

    def __str__(self):
        return f'<TS: {self.week}.{self.TOW}>'

    def __repr__(self):
        return str(self)

    @staticmethod
    def DT2TOW(timestamp: datetime, step=1):
        dt = (timestamp - gps_epoch).total_seconds() + leapS
        TOW = dt % week_seconds
        week = int(dt // week_seconds)
        TOW = math.floor(TOW / step) * step
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
