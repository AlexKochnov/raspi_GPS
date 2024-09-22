from abc import ABCMeta
from cmath import pi
from datetime import datetime

import GPSSingalsParser
from TimeStamp import TimeStamp, BASE_TIME_STAMP


def calc_nmea_checksum(cmd):
    if cmd[0] == '$':
        cmd = cmd[1:]
    if '*' in cmd:
        cmd = cmd.split('*')[0]
    checksum = 0
    for sym in cmd:
        checksum ^= ord(sym)
    return format(checksum, '02X')


def tune_baudRate_message(baudRate):
    cmd = f'$PUBX,41,1,0007,0003,{baudRate},0'
    cmd = cmd + '*' + calc_nmea_checksum(cmd) + '\r\n'
    cmd = cmd.encode('utf-8')
    return cmd

class NmeaMessage(metaclass=ABCMeta):
    receiving_stamp: int or datetime or TimeStamp = None
    header: str = None

    data: dict
    satellites: dict = None

    def __init__(self, receiving_stamp: int or datetime or TimeStamp = BASE_TIME_STAMP()):
        self.receiving_stamp = receiving_stamp
        # self.receiving_stamp = receiving_stamp[1] * Constants.week_seconds + receiving_stamp[0]

    @staticmethod
    def get_subclasses():
        return NmeaMessage.__subclasses__()

    @staticmethod
    def find(header: str) -> type:
        for subclass in NmeaMessage.__subclasses__():
            if subclass.header == header:
                return subclass
        return NmeaMessage

    def __str__(self):
        return f'{self.__class__.__name__}: ' + str(self.to_dict())

    def to_dict(self):
        res = {key: getattr(self, key) for key in self.__dict__}
        if 'satellites' in dir(self):
            if self.satellites is not None:
                res['satellites'] = self.satellites
        return res

    def format_message(self, max_len) -> (str, str):
        S = str(self.to_dict())
        return f'{self.__class__.__name__}:', S[:min(len(S), max_len)]


class PSTMSAT(NmeaMessage):
    header = 'PSTMSAT'

    data: dict = {}

    def __init__(self, msg: str, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        head, SatID, PsR, Freq, Satx, Saty, Satz = msg.split(',')
        self.data = {
            'svId': int(SatID),
            'prMes': int(PsR),
            'freq': int(Freq),
            'x': int(Satx),
            'y': int(Saty),
            'z': int(Satz),
        }

class PSTMTS(NmeaMessage):
    header = 'PSTMTS'

    data: dict = {}

    def __init__(self, msg: str, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        head, dspDat, SatID, PsR, Freq, plf, CN0, ttim, Satdat, Satx, Saty, Satz, Velx, Vely, Velz, src, ac, \
            difdat, drc, predavl, predage, predeph, predtd = msg.split(',')
        if not int(dspDat):
            self.data = None
            return
        self.data = {
            'svId': int(SatID),
            'prMes': float(PsR),
            'freq': float(Freq),
            'plf': int(plf),
            'cno': int(CN0),
            'ttim': float(ttim),
            'satdat': int(Satdat),
            'x': float(Satx),
            'y': float(Saty),
            'z': float(Satz),
            'Vx': float(Velx),
            'Vy': float(Vely),
            'Vz': float(Velz),
            'src': float(src),
            'ac': float(ac),
            'difdat': float(difdat),
            'drc': float(drc),
            'predavl': float(predavl),
            'predage': float(predage),
            'predeph': float(predeph),
            'predtd': float(predtd),
        }

def check_sign(data, n):
    if data >> (n - 1) == 1:
        data -= 2 ** n
    return data

class PSTMALMANAC(NmeaMessage):
    header = 'PSTMALMANAC'

    data: dict = {}

    def __init__(self, msg: str, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        msg = msg.split('*')[0]
        head, SatID, N, bytes_msg = msg.split(',')
        msg = bytes.fromhex(bytes_msg)
        SatID = int(SatID)
        if 1 <= SatID <= 32: # GPS
            self.data = {
                'svId': SatID,
                'svId_dop': msg[0],
                'week': int.from_bytes(msg[1:3], 'little') ,
                'Toa': msg[3] * 2 ** 12,
                'e': int.from_bytes(msg[4:6], 'little') * 2 ** (-21),
                'delta_i': check_sign(int.from_bytes(msg[6:8], 'little'), 16) * 2 ** (-19) * pi,
                'Wdot': check_sign(int.from_bytes(msg[8:10], 'little'), 16) * 2 ** (-38) * pi,
                'sqrtA': int.from_bytes(msg[12:15], 'little') * 2 ** (-11),
                'W0': check_sign(int.from_bytes(msg[16:19], 'little'), 24) * 2 ** (-23) * pi,
                'w': check_sign(int.from_bytes(msg[20:23], 'little'), 24) * 2 ** (-23) * pi,
                'M0': check_sign(int.from_bytes(msg[24:27], 'little'), 24) * 2 ** (-23) * pi,
                'af0': check_sign(((msg[29] & 0x7) << 8) + msg[28], 11) * 2 ** (-20),
                'af1': check_sign(((msg[30] & 0x3F) << 5) + (msg[29] >> 3), 11) * 2 ** (-38),
                'health': (msg[30] >> 6) & 0x1,
                'available': msg[30] >> 7,
            }


class PSTMEPHEM(NmeaMessage):
    header = 'PSTMEPHEM'

    data: dict = {}

    def __init__(self, msg: str, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        msg = msg.split('*')[0]
        head, SatID, N, bytes_msg = msg.split(',')
        msg = bytes.fromhex(bytes_msg)
        SatID = int(SatID)
        if 1 <= SatID <= 32: # GPS
            self.data = {
                'svId': SatID,
                'week': int.from_bytes(msg[0:2], 'little'),
                'Toe': int.from_bytes(msg[2:4], 'little'),
                'Toc': int.from_bytes(msg[4:6], 'little'),
                'accuracy': parse(73, 4),
                # 'CA': parse(71, 2),
                'health': parse(7, 6),
                'IODC': parse2(83, 2, 211, 8),
                'Tgd': parse(197, 8, True) * 2 ** (- 31),
                'af2': parse(241, 8, True) * 2 ** (- 55),
                'af1': parse(249, 16, True) * 2 ** (- 43),
                'af0': parse(271, 22, True) * 2 ** (- 31),
                'IODE1': parse(61, 8),
                'Crs': parse(69, 16, True) * 2 ** (- 5),
                'dn': parse(91, 16, True) * 2 ** (- 43) * pi,
                'M0': parse2(107, 8, 121, 24, True) * 2 ** (- 31) * pi,
                'Cuc': parse(151, 16, True) * 2 ** (- 29),
                'e': parse2(167, 8, 181, 24) * 2 ** (- 33),
                'Cus': parse(211, 16, True) * 2 ** (- 29),
                'sqrtA': parse2(227, 8, 241, 24) * 2 ** (- 19),
                'Cic': parse(61, 16, True) * 2 ** (- 29),
                'W0': parse2(77, 8, 91, 24, True) * 2 ** (- 31) * pi,
                'Cis': parse(121, 16, True) * 2 ** (- 29),
                'i0': parse2(137, 8, 151, 24, True) * 2 ** (- 31) * pi,
                'Crc': parse(181, 16, True) * 2 ** (- 5),
                'w': parse2(197, 8, 211, 24, True) * 2 ** (- 31) * pi,
                'Wdot': parse(241, 24, True) * 2 ** (- 43) * pi,
                'IODE2': parse(271, 8),
                'IDOT': parse(279, 14, True) * 2 ** (- 43) * pi,
            }
