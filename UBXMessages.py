import struct
import traceback
from datetime import datetime
from abc import ABCMeta

import Constants
from TimeStamp import BASE_TIME_STAMP, TimeStamp
from UtilsMessages import GNSS


# BASE_TIME_STAMP = lambda : -1

def calc_ubx_checksum(cmd: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    for c in cmd:
        ck_a = ck_a + c
        ck_b = ck_b + ck_a
    return struct.pack('B', ck_a & 0xff) + struct.pack('B', ck_b & 0xff)


def check_ubx_checksum(cmd: bytes) -> bool:
    return calc_ubx_checksum(cmd[2:-2]) == cmd[-2:]


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


def flag_to_int(flags: bytes) -> int:
    # return sum([(flags[i] & 0xff) << 8 * (len(flags) - 1 - i) for i in range(len(flags))])
    return sum(flags[i] << 8 * i for i in range(len(flags)))
    pass


def get_bytes_from_flag(flags: int, *pattern) -> int:
    return (flags >> min(pattern)) & sum(1 << (x - min(pattern)) for x in pattern)


class UbxMessage(metaclass=ABCMeta):
    receiving_stamp: int or datetime = None
    format: str = None
    header: (int, int) = None

    data: dict
    satellites: dict = None

    def __init__(self, receiving_stamp: int or datetime or TimeStamp = BASE_TIME_STAMP()):
        self.receiving_stamp = receiving_stamp
        # self.receiving_stamp = receiving_stamp[1] * Constants.week_seconds + receiving_stamp[0]


    @staticmethod
    def get_subclasses():
        return UbxMessage.__subclasses__()

    @staticmethod
    def find(header: (int, int)) -> type:
        for subclass in UbxMessage.__subclasses__():
            if subclass.header == header:
                return subclass
        return UbxMessage

    @staticmethod
    def byte_find(clsid: bytes, msgid: bytes) -> type:
        return UbxMessage.find((int.from_bytes(clsid), int.from_bytes(msgid)))

    # def __str__(self):
    #     return f'{type(self).__name__}: {str(self.__dict__)}'

    def __str__(self):
        return f'{self.__class__.__name__}: ' +  str(self.to_dict())

    def to_dict(self):
        res = {key: getattr(self, key) for key in self.__dict__}
        if 'satellites' in dir(self):
            if self.satellites is not None:
                res['satellites'] = self.satellites
        return res

    def format_message(self, max_len) -> (str, str):
        S = str(self.to_dict())
        return f'{self.__class__.__name__}:' , S[:min(len(S), max_len)]


class RXM_RAWX(UbxMessage):
    format = '<dHbBs3B'
    header = (0x02, 0x15)

    data: dict = {}
    satellites: dict = {}

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        rcvTow, week, leapS, numMeas, recStat, *_ = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        recStat = flag_to_int(recStat)
        self.data = {
            'rcvTow': rcvTow,
            'week': week,
            'leapS': leapS,
            'recStat': recStat,
            'leapSec': get_bytes_from_flag(recStat, 0),
            'clkReset': get_bytes_from_flag(recStat, 1),
        }
        for i in range(numMeas):
            prMes, cpMes, doMes, gnssId, svId, _, freqId, locktime, cno, prStedv, cpStedv, doStedv, trkStat, _ = \
                struct.unpack('<ddfBBBBHBssssB', msg[32 * i: 32 * (i + 1)])
            trkStat = flag_to_int(trkStat)
            self.satellites[(svId, GNSS(gnssId))] = {
                'prMes': prMes,
                'cpMes': cpMes,
                'doMes': doMes,
                'freqId': freqId,
                'locktime': locktime,
                'cno': cno,
                'prStedv': 0.01 * 2 ** (flag_to_int(prStedv) & 0x0F),
                'cpStedv': 0.004 * (flag_to_int(cpStedv) & 0x0F),
                'doStedv': 0.002 * 2 ** (flag_to_int(doStedv) & 0x0F),
                'prValid': get_bytes_from_flag(trkStat, 0),
                'cpValid': get_bytes_from_flag(trkStat, 1),
                'halfCyc': get_bytes_from_flag(trkStat, 2),
                'subHalfCyc': get_bytes_from_flag(trkStat, 3),
            }


class NAV_SAT(UbxMessage):
    format = '<LBBBB'
    header = (0x01, 0x35)

    data: dict = {}
    satellites: dict = {}

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        if not msg:
            return
        super().__init__(receiving_TOW)
        iTOW, version, numsSvs, *reversed1 = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.data: dict = {
            'iTOW': iTOW
        }
        for i in range(numsSvs):
            gnssId, svId, cno, elev, azim, prRes, flags = struct.unpack('<BBBbhh4s', msg[12 * i: 12 * (i + 1)])
            flags = flag_to_int(flags)
            self.satellites[(svId, GNSS(gnssId))] = {
                'cno': cno,
                'elev': elev,
                'azim': azim,
                'prRes': prRes * 0.1,
                'qualityInd': get_bytes_from_flag(flags, 0, 1, 2),
                'svUsed': get_bytes_from_flag(flags, 3),
                'health': get_bytes_from_flag(flags, 4, 5),
                'diffCorr': get_bytes_from_flag(flags, 6),
                'smoothed': get_bytes_from_flag(flags, 7),
                'orbitSource': get_bytes_from_flag(flags, 8, 9, 10),
                'ephAvail': get_bytes_from_flag(flags, 11),
                'almAvail': get_bytes_from_flag(flags, 12),
                'anoAvail': get_bytes_from_flag(flags, 13),
                'aopAvail': get_bytes_from_flag(flags, 14),
                'sbasCorrUsed': get_bytes_from_flag(flags, 16),
                'rtcmCorrUsed': get_bytes_from_flag(flags, 17),
                'slasCorrUsed': get_bytes_from_flag(flags, 18),
                'spartnCorrUsed': get_bytes_from_flag(flags, 19),
                'prCorrUsed': get_bytes_from_flag(flags, 20),
                'crCorrUsed': get_bytes_from_flag(flags, 21),
                'doCorrUsed': get_bytes_from_flag(flags, 22),
                'clasCorrUsed': get_bytes_from_flag(flags, 23),
            }


class NAV_ORB(UbxMessage):
    format = '<LBBBB'
    header = (0x01, 0x34)

    data: dict = {}
    satellites: dict = {}

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, version, numsSvs, *reversed1 = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.data = {
            'iTOW': iTOW
        }
        for i in range(numsSvs):
            gnssId, svId, svFlag, eph, alm, otherOrb = struct.unpack('<BBssss', msg[6 * i: 6 * (i + 1)])
            svFlag = flag_to_int(svFlag)
            eph = flag_to_int(eph)
            alm = flag_to_int(alm)
            otherOrb = flag_to_int(otherOrb)
            self.satellites[(svId, GNSS(gnssId))] = {
                'health': svFlag % 4,
                'visibility': (svFlag // 4) % 4,
                'ephUsability': eph % 32,
                'ephSource': eph // 32,
                'almUsability': alm % 32,
                'almSource': alm // 32,
                'anoAopUsability': otherOrb % 32,
                'type': otherOrb // 32,
            }


class RXM_SVSI(UbxMessage):
    format = '<LhBB'
    header = (0x02, 0x20)

    data: dict = {}
    satellites: dict = {}

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, week, numVis, numSV = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.data = {
            'iTOW': iTOW,
            'week': week,
            'numVis': numVis
        }
        for i in range(numSV):
            svId, sv_flag, azim, elev, age_flag = struct.unpack('<Bshbs', msg[+ i * 6: 6 * (i + 1)])
            sv_flag = flag_to_int(sv_flag)
            age_flag = flag_to_int(age_flag)
            self.satellites[(svId, GNSS.GPS)] = {
                'azim': azim,
                'elev': elev,
                'ura': get_bytes_from_flag(sv_flag, 0, 1, 2, 3),
                'healthy': get_bytes_from_flag(sv_flag, 4),
                'ephVal': get_bytes_from_flag(sv_flag, 5),
                'almVal': get_bytes_from_flag(sv_flag, 6),
                'notAvail': get_bytes_from_flag(sv_flag, 7),
                'almAge': (age_flag & 0x0f) - 4,
                'ephAge': ((age_flag & 0xf0) >> 4) - 4,
            }


class RXM_MEASX(UbxMessage):
    format = '<B3sLLL4sLHHH2sHBs8s'
    header = (0x02, 0x14)

    data: dict = {}
    satellites: dict = {}

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        version, _, gpsTOW, gloTOW, bdsTOW, _, qzssTOW, gpsTOWacc , gloTOWacc, bdsTOWacc, _, qzssTOWacc, numSV, flags, _\
            = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        flags = flag_to_int(flags)
        self.data = {
            'gpsTOW': gpsTOW,
            'gloTOW': gloTOW,
            'bdsTOW': bdsTOW,
            'qzssTOW': qzssTOW,
            'gpsTOWacc': gpsTOWacc * 2**(-4),
            'gloTOWacc': gloTOWacc * 2**(-4),
            'bdsTOWacc': bdsTOWacc * 2**(-4),
            'qzssTOWacc': qzssTOWacc * 2**(-4),
            'towSet': get_bytes_from_flag(flags, 0, 1),
        }
        for i in range(numSV):
            gnssId, svId, cno, mpathIndic, dopplerMS, dopplerHz, wholeChips, fracChips, codePhase, intCodePhase, \
                pseuRangeRMSErr, _ = struct.unpack('<BBBBllHHLBB2s', msg[+ i * 24: 24 * (i + 1)])
            self.satellites[(svId, GNSS(gnssId))] = {
                'cno': cno,
                'mpathIndic': mpathIndic,
                'dopplerMS': dopplerMS * 0.04,
                'dopplerHz': dopplerHz * 0.2,
                'wholeChips': wholeChips,
                'fracChips': fracChips,
                'codePhase': codePhase * 2**(-21),
                'intCodePhase': intCodePhase,
                'pseuRangeRMSErr': pseuRangeRMSErr,
            }



class RXM_SFRBX(UbxMessage):
    format = '<BBBBBBBB'
    header = (0x02, 0x13)

    # @staticmethod
    # def ParseSfGPS(msg: bytes, numWords):
    #     sf = struct.unpack('4s' * numWords, msg[:40])
    #     res = ''
    #     for word in sf:
    #         # word = bytes([word[2], word[1], word[0], word[3]])
    #         res += f'{int(word.hex(), 16):032b}'[2:]
    #     return res

    def __init__(self, msg: bytes, receiving_stamp: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_stamp)
        gnssID, svId, _, freqId, numWords, chn, version, _ = \
            struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        if version == 0x02:
            self.chn = None
        msg = msg[struct.calcsize(self.format):]
        if gnssID == 0:
            m = []
            r = []
            for i in range(10):
                m1 = msg[i*4 :4*(i+1)]
                r1 = '{0:024b}'.format((int.from_bytes(m1) >> 6) & 0xFFFFFF)
                m.append(m1)
                r.append(r1)
            a = 0



class NAV_TIMEGPS(UbxMessage):
    format = '<LlhbsL'
    header = (0x01, 0x20)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, fTOW, week, leapS, valid_flag, tAcc = struct.unpack(self.format, msg)
        flags = flag_to_int(valid_flag)
        self.data = {
            'iTOW': iTOW,
            'fTOW': fTOW,
            'weel': week,
            'leapS': leapS,
            'tAcc': tAcc,
            'towValid': get_bytes_from_flag(flags, 0),
            'weekValid': get_bytes_from_flag(flags, 1),
            'leapSValid': get_bytes_from_flag(flags, 2),
        }

class NAV_POSECEF(UbxMessage):
    format = '<LlllL'
    header = (0x01, 0x01)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, ecefX, ecefY, ecefZ, pAcc = struct.unpack(self.format, msg)
        self.data = {
            'iTOW': iTOW,
            'ecefX': ecefX,
            'ecefY': ecefY,
            'ecefZ': ecefZ,
            'pAcc': pAcc,
        }


class NAV_VELECEF(UbxMessage):
    format = '<LlllL'
    header = (0x01, 0x11)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, ecefVX, ecefVY, ecefVZ, sAcc = struct.unpack(self.format, msg)
        self.data = {
            'iTOW': iTOW,
            'ecefX': ecefVX,
            'ecefY': ecefVY,
            'ecefZ': ecefVZ,
            'pAcc': sAcc,
        }



class AID_EPH(UbxMessage):
    format = '<LL'
    header = (0x0B, 0x31)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        svId, HOW = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.stamp = (svId, GNSS.GPS)
        if HOW == 0:
            self.data = None
            return
        [week, accuracy, health, IODC, Tgd, Toc, af2, af1, af0] = GPSParser.parse(sf2bin(msg[:32]), 1)
        [IODE1, Crs, dn, M0, Cuc, e, Cus, sqrtA, Toe] = GPSParser.parse(sf2bin(msg[32:64]), 2)
        [Cic, W0, Cis, i0, Crc, w, Wdot, IODE2, IDOT] = GPSParser.parse(sf2bin(msg[64:]), 3)
        self.data = {
            'week': week,
            'Toe': Toe,
            'Toc': Toc,
            'IODE1': IODE1,
            'IODE2': IODE2,
            'IODC': IODC,
            'IDOT': IDOT,
            'Wdot': Wdot,
            'Crs': Crs,
            'Crc': Crc,
            'Cus': Cus,
            'Cuc': Cuc,
            'Cis': Cis,
            'Cic': Cic,
            'dn': dn,
            'i0': i0,
            'e': e,
            'sqrtA': sqrtA,
            'M0': M0,
            'W0': W0,
            'w': w,
            'Tgd': Tgd,
            'af2': af2,
            'af1': af1,
            'af0': af0,
            'health': health,
            'accuracy': accuracy,
        }


class AID_ALM(UbxMessage):
    format = '<LL'
    header = (0x0B, 0x30)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        svId, week = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.stamp = (svId, GNSS.GPS)
        if week == 0:
            self.data = None
            return
        [SV_ID, Data_ID, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health] = \
            GPSParser.parse(sf2bin(msg), 5)
        self.data = {
            'week': week,
            'Toa': Toa,
            'e': e,
            'delta_i': delta_i,
            'Wdot': Wdot,
            'sqrtA': sqrtA,
            'W0': W0,
            'w': w,
            'M0': M0,
            'af0': af0,
            'af1': af1,
            'health': health,
            'Data_ID': Data_ID,
        }


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
        week = bin2dec(subframe[60:70]) + 1024 * 2
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
