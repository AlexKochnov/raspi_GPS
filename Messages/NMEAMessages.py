from abc import ABCMeta
from cmath import pi
from datetime import datetime

from Utils.Constants import tz_utc
from Utils.GNSS import GNSS
from Utils.TimeStamp import TimeStamp, BASE_TIME_STAMP

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
            if subclass.header in header:
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

    def format_message(self):
        return f'<strong>{self.__class__.__name__}:</strong>' + str(self.to_dict())

    @staticmethod
    def get_head(msg: str):
        return msg.split(',')[0]

class RMC(NmeaMessage):
    header = 'RMC'

    data: dict = {}

    def __init__(self, msg: str, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        msg = msg.split('*')[0]
        head, time, status, lat, NS, lon, EW, spd, cog, date, mv, mvEW, *other = msg.split(',') #posMode, navStatus

        if isinstance(self.receiving_stamp, TimeStamp):
            try:
                dt = datetime.strptime(date + time.split('.')[0], '%d%m%y%H%M%S')
                dt = dt.replace(tzinfo=tz_utc)
                self.receiving_stamp = TimeStamp(datetime=dt)
            except:
                pass
        a=0
        # self.data = {
        # }
        # if status == 'A':
        #     self.data |= {'lat'}

class GGA(NmeaMessage):
    header = 'GGA'

    data: dict = {}

    def __init__(self, msg: str, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        msg = msg.split('*')[0]
        head, time, lat, NS, lon, EW, quality, numSV, HDOP, alt, altUnit, sep, sepUnit, *other = msg.split(',') # diffAge, diffStation
        LAT = float(lat)
        LON = float(lon)
        self.data = {
            'lat': (LAT//100 + (LAT % 100) / 60) * (1 if NS == 'N' else -1),
            'lon': (LON//100 + (LON % 100) / 60) * (1 if EW == 'E' else -1),
            'alt': float(alt)
        }

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
            difdat, drc, *other = msg.split(',') # predavl, predage, predeph, predtd
        if not int(dspDat):
            self.data = None
            return
        SatID = int(SatID)
        if int(Satdat) and int(plf):
            if 1 <= SatID <= 32: # GPS
                self.data = {
                    'svId': SatID,
                    'gnssId': GNSS.GPS,
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
                    # 'predavl': float(predavl),
                    # 'predage': float(predage),
                    # 'predeph': float(predeph),
                    # 'predtd': float(predtd),
                }
        a=0

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
                'gnssId': GNSS.GPS,
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
        a=0


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
                'gnssId': GNSS.GPS,
                'week': int.from_bytes(msg[0:2], 'little'),
                'Toe': int.from_bytes(msg[2:4], 'little') * 16,
                'Toc': int.from_bytes(msg[4:6], 'little') * 16,
                'IODE1': msg[6],
                'IODE2': msg[7],
                'IODC': ((msg[9] & 0x3) << 8) + msg[8], # check_sign(((msg[30] & 0x3F) << 5) + (msg[29] >> 3), 11) * 2 ** (-38)
                'IDOT': check_sign(((msg[10] & 0x3F) << 8) + msg[9], 14) * 2 ** (- 43) * pi,
                'Wdot': check_sign(int.from_bytes(msg[12:15], 'little'), 24) * 2 ** (- 43) * pi,
                'Crs': check_sign(int.from_bytes(msg[16:18], 'little'), 16) * 2 ** (- 5),
                'Crc': check_sign(int.from_bytes(msg[18:20], 'little'), 16) * 2 ** (- 5),
                'Cus': check_sign(int.from_bytes(msg[20:22], 'little'), 16) * 2 ** (- 29),
                'Cuc': check_sign(int.from_bytes(msg[22:24], 'little'), 16) * 2 ** (- 29),
                'Cis': check_sign(int.from_bytes(msg[24:26], 'little'), 16) * 2 ** (- 29),
                'Cic': check_sign(int.from_bytes(msg[26:28], 'little'), 16) * 2 ** (- 29),
                'dn': check_sign(int.from_bytes(msg[28:30], 'little'), 16) * 2 ** (- 43) * pi,
                'i0': check_sign(int.from_bytes(msg[32:36], 'little'), 32) * 2 ** (- 31) * pi,
                'e': int.from_bytes(msg[36:40], 'little') * 2 ** (- 33),
                'sqrtA': int.from_bytes(msg[40:44], 'little') * 2 ** (- 19),
                'M0': check_sign(int.from_bytes(msg[44:48], 'little'), 32) * 2 ** (- 31) * pi,
                'W0': check_sign(int.from_bytes(msg[48:52], 'little'), 32) * 2 ** (- 31) * pi,
                'w': check_sign(int.from_bytes(msg[52:56], 'little'), 32) * 2 ** (- 31) * pi,
                'Tgd': check_sign(msg[56], 8) * 2 ** (- 31),
                'af2': check_sign(msg[57], 8) * 2 ** (- 55),
                'af1': check_sign(int.from_bytes(msg[58:60], 'little'), 16) * 2 ** (- 43),
                'af0': check_sign(((((msg[62] & 0x3F) << 8) + msg[61]) << 8) + msg[60], 22) * 2 ** (-31),
                'health': (msg[63] >> 2) & 0x1,
                'accuracy': msg[63] >> 4,
            }
