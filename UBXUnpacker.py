import struct
from datetime import datetime
from abc import ABCMeta

from Satellites import *

# для отладки
from tabulate import tabulate

from UBXUtils import flag_to_int, get_bytes_from_flag


class Message(metaclass=ABCMeta):
    receiving_time: datetime = None
    format: str = None
    header: (int, int) = None

    def __init__(self, receiving_time: datetime = None):
        self.receiving_time = receiving_time or datetime.now()

    @staticmethod
    def get_subclasses():
        return Message.__subclasses__()

    @staticmethod
    def find(header: (int, int)) -> type:
        for subclass in Message.__subclasses__():
            if subclass.header == header:
                return subclass
        return Message

    @staticmethod
    def byte_find(clsid: bytes, msgid: bytes) -> type:
        return Message.find((int.from_bytes(clsid), int.from_bytes(msgid)))

    def __str__(self):
        return f'{type(self).__name__}: {str(self.__dict__)}'

class Command(metaclass=ABCMeta):
    pass


def sf2bin(msg: bytes) -> str:
    sf = struct.unpack('4s' * 8, msg)
    res = '0' * 60
    for word in sf:
        word = bytes([word[2], word[1], word[0], word[3]])
        res += f'{int(word.hex(), 16):032b}'[:-2]
    return res


def bin2dec(binaryStr: str) -> int:
    return int(binaryStr, 2)


def twosComp2dec(binaryStr: str) -> int:
    # функция с проверкой знака
    intNumber = int(binaryStr, 2)
    if binaryStr[0] == '1':
        intNumber -= 2 ** len(binaryStr)
    return intNumber


# TODO: add RXM-RAWX and RAWX (sat)
class RXM_RAWX(Message):
    format = '<dHbBs3B'
    header = (0x02, 0x15)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.rcvTow, self.week, self.leapS, numMeas, recStat, *_ =\
            struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        recStat = flag_to_int(recStat)
        self.leapSec = get_bytes_from_flag(recStat, 0)
        self.clkReset = get_bytes_from_flag(recStat, 1)
        msg = msg[struct.calcsize(self.format):]
        self.rawx = dict()
        for i in range(numMeas):
            prMes, cpMes, doMes, gnssId, svId, _, freqId, locktime, cno, prStedv, cpStedv, doStedv, trkStat, _ =\
            struct.unpack('<ddfBBBBHBssssB', msg[32 * i: 32 * (i + 1)])
            self.rawx[(gnssId, svId)] =\
                RAWX(prMes, cpMes, doMes, freqId, locktime, cno, prStedv, cpStedv, doStedv, trkStat)
        a = 0


class NAV_SAT(Message):
    format = '<LBBBB'
    header = (0x01, 0x35)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, version, numsSvs, *reversed1 = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.sat = dict()
        for i in range(numsSvs):
            gnssId, svId, cno, elev, azim, prRes, flags = struct.unpack('<BBBbhh4s', msg[12 * i: 12 * (i + 1)])
            self.sat[(gnssId, svId)] = SAT(cno, elev, azim, prRes, flags)


class NAV_ORB(Message):
    format = '<LBBBB'
    header = (0x01, 0x34)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, version, numsSvs, *reversed1 = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.orb = dict()
        for i in range(numsSvs):
            gnssId, svId, svFlag, eph, alm, otherOrb = struct.unpack('<BBssss', msg[6 * i: 6 * (i + 1)])
            self.orb[(gnssId, svId)] = ORB(svFlag, eph, alm, otherOrb)


class NAV_TIMEGPS(Message):
    format = '<LlhbsL'
    header = (0x01, 0x20)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, self.fTOW, self.week, self.leapS, valid_flag, self.tAcc = struct.unpack(self.format, msg)
        self.TOW = self.iTOW * 1e-3 + self.fTOW * 1e-9
        flags = flag_to_int(valid_flag)
        self.Valid = {
            'TOW': get_bytes_from_flag(flags, 0),
            'week': get_bytes_from_flag(flags, 1),
            'leapS': get_bytes_from_flag(flags, 2),
        }


class NAV_POSECEF(Message):
    format = '<LlllL'
    header = (0x01, 0x01)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, self.ecefX, self.ecefY, self.ecefZ, self.pAcc = struct.unpack(self.format, msg)
        # self.LLA = ecef2lla(self.ecefX/100, self.ecefY/100, self.ecefZ/100)


class NAV_VELECEF(Message):
    format = '<LlllL'
    header = (0x01, 0x11)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, self.ecefVX, self.ecefVY, self.ecefVZ, self.sAcc = struct.unpack(self.format, msg)


class RXM_SVSI(Message):
    format = '<LhBB'
    header = (0x02, 0x20)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, self.week, self.numVis, numSV = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.svsi = dict()
        for i in range(numSV):
            svId, sv_flag, azim, elev, age_flag = struct.unpack('<Bshbs', msg[+ i * 6: 6 * (i + 1)])
            self.svsi[(GNSS.GPS, svId)] = SVSI(sv_flag, azim, elev, age_flag)


class AID_EPH(Message):
    format = '<LL'
    header = (0x0B, 0x31)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.svId, self.HOW = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        if self.HOW == 0:
            self.eph = None
            return
        [week, accuracy, health, IODC, Tgd, Toc, af2, af1, af0] = GPSParser.parse(sf2bin(msg[:32]), 1)
        [IODE1, Crs, dn, M0, Cuc, e, Cus, sqrtA, Toe] = GPSParser.parse(sf2bin(msg[32:64]), 2)
        [Cic, W0, Cis, i0, Crc, w, Wdot, IODE2, IDOT] = GPSParser.parse(sf2bin(msg[64:]), 3)
        self.eph = [self.svId, week, Toe, Toc, IODE1, IODE2, IODC, IDOT, Wdot, Crs, Crc, Cus, Cuc, Cis, Cic, dn, i0, e,
                    sqrtA, M0, W0, w, Tgd, af2, af1, af0, health, accuracy, self.receiving_time]


class AID_ALM(Message):
    format = '<LL'
    header = (0x0B, 0x30)

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.svId, self.week = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        if self.week == 0:
            self.alm = None
            return
        [SV_ID, Data_ID, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health] = \
            GPSParser.parse(sf2bin(msg), 5)
        self.alm = [SV_ID, self.week, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health, Data_ID,
                    self.receiving_time]


class RXM_SFRBX(Message):
    format = '<BBBBBBBB'
    header = (0x02, 0x13)

    @staticmethod
    def ParseSfGPS(msg: bytes, numWords):
        sf = struct.unpack('4s' * numWords, msg[:40])
        res = ''
        for word in sf:
            # word = bytes([word[2], word[1], word[0], word[3]])
            res += f'{int(word.hex(), 16):032b}'[2:]
        return res

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.gnssID, self.svId, _, self.freqId, numWords, self.chn, version, _ = \
            struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        if version == 0x02:
            self.chn = None
        msg = msg[struct.calcsize(self.format):]

        self.plb = msg

        if self.gnssID == GNSS.GPS.value:
            subframe = self.ParseSfGPS(msg, numWords)
            HOW = subframe[30:60]
            TLM = subframe[0:30]
            sfId = int(HOW[19:22], 2)
            # match sfId:
            #     case 1:
            # [week, accuracy, health, IODC, Tgd, Toc, af2, af1, af0]\
            data = GPSParser.parse(subframe, 1)
            h = ['week', 'accuracy', 'health', 'IODC', 'Tgd', 'Toc', 'af2', 'af1', 'af0']
            print(tabulate([data], headers=h))
            print('\n')
            # case 2:
            # [IODE1, Crs, dn, M0, Cuc, e, Cus, sqrtA, Toe] \
            data = GPSParser.parse(subframe, 2)
            h = ['IODE1', 'Crs', 'dn', 'M0', 'Cuc', 'e', 'Cus', 'sqrtA', 'Toe']
            print(tabulate([data], headers=h))
            print('\n')
            # case 3:
            # [Cic, W0, Cis, i0, Crc, w, Wdot, IODE2, IDOT]
            data = GPSParser.parse(subframe, 3)
            h = ['Cic', 'W0', 'Cis', 'i0', 'Crc', 'w', 'Wdot', 'IODE2', 'IDOT']
            print(tabulate([data], headers=h))
            print('\n')
            # case 5:
            h = ['SV_ID', 'Data_ID', 'Toa', 'e', 'delta_i', 'Wdot', 'sqrtA', 'W0', 'w', 'M0', 'af0', 'af1', 'health']
            data = GPSParser.parse(subframe, 5)
            print(tabulate([data], headers=h))
            print('\n')
            # case _:
            #     data = None
            # if data:
            #     print(tabulate(data, headers=h))
            print(f'SFRBX : {self.gnssID} : {self.svId} : {sfId} : {subframe}')
        print('\n')
        a = 0


class GPSParser:
    @staticmethod
    def parse(subframe, subframe_number: int) -> list:
        return {
            1: GPSParser.ParseSf1,
            2: GPSParser.ParseSf2,
            3: GPSParser.ParseSf3,
            5: GPSParser.ParseSf5,
        }[subframe_number](subframe)

    @staticmethod
    def ParseSf1(subframe):
        week = bin2dec(subframe[60:70]) + 0 * 1024 * 2
        accuracy = bin2dec(subframe[72:76])
        health = bin2dec(subframe[76:82])
        IODC = bin2dec(subframe[82:84] + subframe[196:204])
        Tgd = twosComp2dec(subframe[195:204]) * 2 ** (- 31)
        Toc = bin2dec(subframe[218:234]) * 2 ** 4
        af2 = twosComp2dec(subframe[240:248]) * 2 ** (- 55)
        af1 = twosComp2dec(subframe[248:264]) * 2 ** (- 43)
        af0 = twosComp2dec(subframe[270:292]) * 2 ** (- 31)
        return [week, accuracy, health, IODC, Tgd, Toc, af2, af1, af0]

    @staticmethod
    def ParseSf2(subframe):
        IODE1 = bin2dec(subframe[60:68])
        Crs = twosComp2dec(subframe[68:84]) * 2 ** (- 5)
        dn = twosComp2dec(subframe[90:106]) * 2 ** (- 43)
        M0 = twosComp2dec(subframe[106:114] + subframe[120:144]) * 2 ** (- 31)
        Cuc = twosComp2dec(subframe[150:166]) * 2 ** (- 29)
        e = bin2dec(subframe[166:174] + subframe[180:204]) * 2 ** (- 33)
        Cus = twosComp2dec(subframe[210:226]) * 2 ** (- 29)
        sqrtA = bin2dec(subframe[226:234] + subframe[240:264]) * 2 ** (- 19)
        Toe = bin2dec(subframe[270:286]) * 2 ** 4
        return [IODE1, Crs, dn, M0, Cuc, e, Cus, sqrtA, Toe]

    @staticmethod
    def ParseSf3(subframe):
        Cic = twosComp2dec(subframe[60:76]) * 2 ** (- 29)
        W0 = twosComp2dec(subframe[76:84] + subframe[90:114]) * 2 ** (- 31)
        Cis = twosComp2dec(subframe[120:136]) * 2 ** (- 29)
        i0 = twosComp2dec(subframe[136:144] + subframe[150:174]) * 2 ** (- 31)
        Crc = twosComp2dec(subframe[180:196]) * 2 ** (- 5)
        w = twosComp2dec(subframe[196:204] + subframe[210:234]) * 2 ** (- 31)
        Wdot = twosComp2dec(subframe[240:264]) * 2 ** (- 43)
        IODE2 = bin2dec(subframe[270:278])
        IDOT = twosComp2dec(subframe[278:292]) * 2 ** (- 43)
        return [Cic, W0, Cis, i0, Crc, w, Wdot, IODE2, IDOT]

    @staticmethod
    def ParseSf5(subframe):
        Data_ID = bin2dec(subframe[60:62])
        SV_ID = bin2dec(subframe[62:68])
        e = bin2dec(subframe[68:84]) * 2 ** (-21)
        Toa = bin2dec(subframe[90:98]) * 2 ** 12
        delta_i = twosComp2dec(subframe[98:114]) * 2 ** (-19)
        Wdot = twosComp2dec(subframe[120:136]) * 2 ** (-38)
        health = bin2dec(subframe[136:144])
        sqrtA = bin2dec(subframe[150:174]) * 2 ** (-11)
        W0 = twosComp2dec(subframe[180:204]) * 2 ** (-23)
        w = twosComp2dec(subframe[210:234]) * 2 ** (-23)
        M0 = twosComp2dec(subframe[240:264]) * 2 ** (-23)
        af1 = twosComp2dec(subframe[278:289]) * 2 ** (-38)
        af0 = twosComp2dec(subframe[270:278] + subframe[289:292]) * 2 ** (-20)
        return [SV_ID, Data_ID, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health]
