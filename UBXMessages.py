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
        version, _, gpsTOW, gloTOW, bdsTOW, _, qzssTOW, gpsTOWacc, gloTOWacc, bdsTOWacc, _, qzssTOWacc, numSV, flags, _ \
            = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        flags = flag_to_int(flags)
        self.data = {
            'gpsTOW': gpsTOW,
            'gloTOW': gloTOW,
            'bdsTOW': bdsTOW,
            'qzssTOW': qzssTOW,
            'gpsTOWacc': gpsTOWacc * 2 ** (-4),
            'gloTOWacc': gloTOWacc * 2 ** (-4),
            'bdsTOWacc': bdsTOWacc * 2 ** (-4),
            'qzssTOWacc': qzssTOWacc * 2 ** (-4),
            'towSet': get_bytes_from_flag(flags, 0, 1),
        }
        for i in range(numSV):
            gnssId, svId, cno, mpathIndic, dopplerMS, dopplerHz, wholeChips, fracChips, codePhase, intCodePhase, \
                pseuRangeRMSErr, _ = struct.unpack('<BBBBllHHLBB2s', msg[+ i * 24: 24 * (i + 1)])
            self.satellites[(svId, GNSS(gnssId))] = {
                # 'cNo': cno, # carrier noise ratio (0..63) - коэффициент -> не то
                'mpathIndic': mpathIndic,
                'dopplerMS': dopplerMS * 0.04,
                'dopplerHz': dopplerHz * 0.2,
                'wholeChips': wholeChips,
                'fracChips': fracChips,
                'codePhase': codePhase * 2 ** (-21),
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
                m1 = msg[i * 4:4 * (i + 1)]
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


def gps_join_sf(msg: bytes, reverse_mod: str = 'FULL') -> int:
    if reverse_mod.upper() == 'AID':
        aid_reverse = lambda word: [word[2], word[1], word[0], word[3]]
        words = [0, 0] + [(int.from_bytes(aid_reverse(msg[i:i + 4])) >> 2) & 0x3FFFFFFF for i in range(0, len(msg), 4)]
    elif reverse_mod == 'FULL':
        words = [int.from_bytes(msg[i:i + 4][::-1]) & 0x3FFFFFFF for i in range(0, len(msg), 4)]
    else:
        # raise TypeError(reverse_mod)
        return 0
    return sum((word << 30 * (10 - i - 1)) for i, word in enumerate(words))



# старые функции

def gps_sf2bin(msg: bytes) -> str:
    sf = struct.unpack('4s' * 8, msg)
    res = '0' * 60
    for word in sf:
        word = bytes([word[2], word[1], word[0], word[3]])
        res += f'{int(word.hex(), 16):032b}'[:-2]
    return res


def gps_bin2dec(binaryStr: str) -> int:
    return int(binaryStr, 2)


def gps_twosComp2dec(binaryStr: str) -> int:
    # функция с проверкой знака
    intNumber = int(binaryStr, 2)
    if binaryStr[0] == '1':
        intNumber -= 2 ** len(binaryStr)
    return intNumber


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
        data1 = GPSParser.parse(gps_join_sf(msg[:32], 'AID'), 1)
        data2 = GPSParser.parse(gps_join_sf(msg[32:64], 'AID'), 2)
        data3 = GPSParser.parse(gps_join_sf(msg[64:], 'AID'), 3)
        self.data = data1 | data2 | data3


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

        self.data = GPSParser.parse(gps_join_sf(msg, 'AID'), 5)


def get_bits_gps(data, l, n, sign=False):
    res = ((data >> (300 - (l+n-1))) & ((1 << n) - 1))
    if sign is True and res >> (n-1) == 1:
        res -= 2 ** n
    return res


class GPSParser:

    @staticmethod
    def get_parse_function(subframe: int):
        return lambda l, n, sign=False: get_bits_gps(subframe, l, n, sign)

    @staticmethod
    def get_parse_function2(subframe: int):
        def parse2(l1, n1, l2, n2, sign=False):
            d1 = get_bits_gps(subframe, l1, n1, False)
            d2 = get_bits_gps(subframe, l2, n2, False)
            res = (d1 << n2) + d2
            if sign is True and res >> (n1+n2-1) == 1:
                res -= 2 ** (n1 + n2)
            return res
        return parse2

    @staticmethod
    def parse(subframe_data: int, subframe_number: int = None) -> dict:
        if subframe_number is not None:
            return {
                1: GPSParser.parse_sf_1,
                2: GPSParser.parse_sf_2,
                3: GPSParser.parse_sf_3,
                5: GPSParser.parse_sf_alm,
            }[subframe_number](subframe_data, full=False)
        pass

    @staticmethod
    def get_head_stats(subframe_data, full) -> dict:
        if not full:
            return {}
        return {
            # TODO: Доделать
        }

    @staticmethod
    def parse_sf_1(subframe, full=True) -> dict:
        parse = GPSParser.get_parse_function(subframe)
        parse2 = GPSParser.get_parse_function2(subframe)
        return {
            'week': parse(61, 10) + 1024 * 2,
            'accuracy': parse(73, 4),
            'health': parse(7, 6),
            'IODC': parse2(83, 2, 211, 8),
            'Tgd': parse(197, 8, True) * 2 ** (- 31),
            'Toc': parse(219, 16) * 2 ** 4,
            'af2': parse(241, 8, True) * 2 ** (- 55),
            'af1': parse(249, 16, True) * 2 ** (- 43),
            'af0': parse(271, 22, True) * 2 ** (- 31),
        } | GPSParser.get_head_stats(subframe, full)

    @staticmethod
    def parse_sf_2(subframe, full=True) -> dict:
        parse = GPSParser.get_parse_function(subframe)
        parse2 = GPSParser.get_parse_function2(subframe)
        return {
            'IODE1': parse(61, 8),
            'Crs': parse(69, 16, True) * 2 ** (- 5),
            'dn': parse(91, 16, True) * 2 ** (- 43),
            'M0': parse2(107, 8, 121, 24, True) * 2 ** (- 31),
            'Cuc': parse(151, 16, True) * 2 ** (- 29),
            'e': parse2(167, 8, 181, 24) * 2 ** (- 33),
            'Cus': parse(211, 16, True) * 2 ** (- 29),
            'sqrtA': parse2(227, 8, 241, 24) * 2 ** (- 19),
            'Toe': parse(271, 16) * 2 ** 4,
        } | GPSParser.get_head_stats(subframe, full)

    @staticmethod
    def parse_sf_3(subframe, full=True) -> dict:
        parse = GPSParser.get_parse_function(subframe)
        parse2 = GPSParser.get_parse_function2(subframe)
        return {
            'Cic': parse(61, 16, True) * 2 ** (- 29),
            'W0': parse2(77, 8, 91, 24, True) * 2 ** (- 31),
            'Cis': parse(121, 16, True) * 2 ** (- 29),
            'i0': parse2(137, 8, 151, 24, True) * 2 ** (- 31),
            'Crc': parse(181, 16, True) * 2 ** (- 5),
            'w': parse2(197, 8, 211, 24, True) * 2 ** (- 31),
            'Wdot': parse(241, 24, True) * 2 ** (- 43),
            'IODE2': parse(271, 8),
            'IDOT': parse(279, 14, True) * 2 ** (- 43),
        } | GPSParser.get_head_stats(subframe, full)

    @staticmethod
    def parse_sf_alm(subframe, full=True) -> dict:
        parse = GPSParser.get_parse_function(subframe)
        parse2 = GPSParser.get_parse_function2(subframe)
        return {
            'Data_ID': parse(61, 2),
            'SV_ID': parse(63, 6),
            'e': parse(69, 16) * 2 ** (-21),
            'Toa': parse(91, 8) * 2 ** 12,
            'delta_i': parse(99, 16, True) * 2 ** (-19),
            'Wdot': parse(121, 16, True) * 2 ** (-38),
            'health': parse(137, 8),
            'sqrtA': parse(151, 24) * 2 ** (-11),
            'W0': parse(181, 24, True) * 2 ** (-23),
            'w': parse(211, 24, True) * 2 ** (-23),
            'M0': parse(241, 24, True) * 2 ** (-23),
            'af1': parse(279, 11, True) * 2 ** (-38),
            'af0': parse2(271, 8, 290, 3, True) * 2 ** (-20),
        } | GPSParser.get_head_stats(subframe, full)


class GPSParser1:
    @staticmethod
    def parse(subframe, subframe_number: int) -> list:
        return {
            1: GPSParser1.ParseSf1,
            2: GPSParser1.ParseSf2,
            3: GPSParser1.ParseSf3,
            5: GPSParser1.ParseSf5,
        }[subframe_number](subframe)

    @staticmethod
    def ParseSf1(subframe):
        week = gps_bin2dec(subframe[60:70]) + 1024 * 2
        accuracy = gps_bin2dec(subframe[72:76])
        health = gps_bin2dec(subframe[76:82])
        IODC = gps_bin2dec(subframe[82:84] + subframe[196:204])
        Tgd = gps_twosComp2dec(subframe[195:204]) * 2 ** (- 31)
        Toc = gps_bin2dec(subframe[218:234]) * 2 ** 4
        af2 = gps_twosComp2dec(subframe[240:248]) * 2 ** (- 55)
        af1 = gps_twosComp2dec(subframe[248:264]) * 2 ** (- 43)
        af0 = gps_twosComp2dec(subframe[270:292]) * 2 ** (- 31)
        return [week, accuracy, health, IODC, Tgd, Toc, af2, af1, af0]

    @staticmethod
    def ParseSf2(subframe):
        IODE1 = gps_bin2dec(subframe[60:68])
        Crs = gps_twosComp2dec(subframe[68:84]) * 2 ** (- 5)
        dn = gps_twosComp2dec(subframe[90:106]) * 2 ** (- 43)
        M0 = gps_twosComp2dec(subframe[106:114] + subframe[120:144]) * 2 ** (- 31)
        Cuc = gps_twosComp2dec(subframe[150:166]) * 2 ** (- 29)
        e = gps_bin2dec(subframe[166:174] + subframe[180:204]) * 2 ** (- 33)
        Cus = gps_twosComp2dec(subframe[210:226]) * 2 ** (- 29)
        sqrtA = gps_bin2dec(subframe[226:234] + subframe[240:264]) * 2 ** (- 19)
        Toe = gps_bin2dec(subframe[270:286]) * 2 ** 4
        return [IODE1, Crs, dn, M0, Cuc, e, Cus, sqrtA, Toe]

    @staticmethod
    def ParseSf3(subframe):
        Cic = gps_twosComp2dec(subframe[60:76]) * 2 ** (- 29)
        W0 = gps_twosComp2dec(subframe[76:84] + subframe[90:114]) * 2 ** (- 31)
        Cis = gps_twosComp2dec(subframe[120:136]) * 2 ** (- 29)
        i0 = gps_twosComp2dec(subframe[136:144] + subframe[150:174]) * 2 ** (- 31)
        Crc = gps_twosComp2dec(subframe[180:196]) * 2 ** (- 5)
        w = gps_twosComp2dec(subframe[196:204] + subframe[210:234]) * 2 ** (- 31)
        Wdot = gps_twosComp2dec(subframe[240:264]) * 2 ** (- 43)
        IODE2 = gps_bin2dec(subframe[270:278])
        IDOT = gps_twosComp2dec(subframe[278:292]) * 2 ** (- 43)
        return [Cic, W0, Cis, i0, Crc, w, Wdot, IODE2, IDOT]

    @staticmethod
    def ParseSf5(subframe):
        Data_ID = gps_bin2dec(subframe[60:62])
        SV_ID = gps_bin2dec(subframe[62:68])
        e = gps_bin2dec(subframe[68:84]) * 2 ** (-21)
        Toa = gps_bin2dec(subframe[90:98]) * 2 ** 12
        delta_i = gps_twosComp2dec(subframe[98:114]) * 2 ** (-19)
        Wdot = gps_twosComp2dec(subframe[120:136]) * 2 ** (-38)
        health = gps_bin2dec(subframe[136:144])
        sqrtA = gps_bin2dec(subframe[150:174]) * 2 ** (-11)
        W0 = gps_twosComp2dec(subframe[180:204]) * 2 ** (-23)
        w = gps_twosComp2dec(subframe[210:234]) * 2 ** (-23)
        M0 = gps_twosComp2dec(subframe[240:264]) * 2 ** (-23)
        af1 = gps_twosComp2dec(subframe[278:289]) * 2 ** (-38)
        af0 = gps_twosComp2dec(subframe[270:278] + subframe[289:292]) * 2 ** (-20)
        return [SV_ID, Data_ID, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health]
