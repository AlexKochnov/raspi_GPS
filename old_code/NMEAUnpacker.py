from abc import ABCMeta
from datetime import datetime

from Satellites import SVSTATUS


class NmeaMessage(metaclass=ABCMeta):
    receiving_time: datetime = None
    header: (int, int) = None
    nmea_type: str = None

    def __init__(self, receiving_time: datetime = None):
        self.receiving_time = receiving_time or datetime.now()

    @staticmethod
    def get_nmea_type(msg: str):
        pubx, msgid, *other = msg.split(',')
        return msgid

    @staticmethod
    def get_subclasses():
        return NmeaMessage.__subclasses__()

    @staticmethod
    def find(finder: (int, int) or str) -> type:
        attr = 'nmea_type' if isinstance(finder, str) else 'header'
        for subclass in NmeaMessage.__subclasses__():
            if getattr(subclass, attr) == finder:
                return subclass
        return NmeaMessage

    @staticmethod
    def byte_find(clsid: bytes, msgid: bytes) -> type:
        return NmeaMessage.find((int.from_bytes(clsid), int.from_bytes(msgid)))

    def __str__(self):
        return f'{type(self).__name__}: {str(self.__dict__)}'

def str2int(s: str) -> int or None:
    if len(s):
        return int(s)
    return None

class PUBX_SVSTATUS(NmeaMessage):
    header = (0xF1, 0x03)
    nmea_type = '03'
    attr = 'rawx'
    stats = {
        '-': 0, # not used
        'U': 1, # used in solution
        'e': 2, # Ephemeris available, but not used for navigation
    }

    def __init__(self, msg: str, receiving_time: datetime = None):
        super().__init__(receiving_time)
        msg = msg.split('*')[0]
        pubx, msgId, n, *other = msg.split(',')
        self.svstatus = dict()
        for i in range(int(n)):
            sv, s, az, el, cno, lck, *other = other
            ## TODO: не факт, что это GPS, погут быить перекрытия с Galileo и BeiDou (см. док.)
            self.svstatus[(0, int(sv))] = SVSTATUS(status=self.stats[s], azim=str2int(az), elev=str2int(el),
                                              cno=str2int(cno), lck=str2int(lck))

