import struct
from datetime import datetime
from abc import ABCMeta

import GPSSingalsParser, GLONASSSignalsParser
from Utils.TimeStamp import BASE_TIME_STAMP, TimeStamp
from Utils.GNSS import GNSS


# BASE_TIME_STAMP = lambda : -1

def calc_prRMSer(index):
    index = int(index)
    y = (index >> 3) & 0x7
    x = (index) & 0x7
    RMSer = 0.5 * (1 + float(x) / 8.0) * 2.0 ** y
    return RMSer


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


def get_stamps(obj, svId: int, gnssId: GNSS) -> dict:
    return {'svId': svId, 'gnssId': gnssId} | {f'{obj.__class__.__name__}_stamp': obj.receiving_stamp}

# @dataclass
class UbxMessage(metaclass=ABCMeta):
    receiving_stamp: int or datetime = None
    format: str = None
    header: (int, int) = None

    data: dict = None
    satellites: dict = None

    def __init__(self, receiving_stamp: int or datetime or TimeStamp = BASE_TIME_STAMP()):
        self.receiving_stamp = receiving_stamp
        # self.receiving_stamp = receiving_stamp[1] * Constants.week_seconds + receiving_stamp[0]

    # def __post_init__(self):
    #     if self.data:
    #         if 'TOW' in self.data:
    #             self.receiving_stamp.TOW = self.data['TOW']
    #         if 'week' in self.data:
    #             self.receiving_stamp.week = self.data['week']

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
    satellites: list = []

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        self.data: dict = {}
        self.satellites: list = []
        rcvTow, week, leapS, numMeas, recStat, *_ = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        recStat = flag_to_int(recStat)
        self.data = {
            'rcvTow': rcvTow,
            'week': week,
            'leapS': leapS,
            # 'recStat': recStat,
            'leapSec': get_bytes_from_flag(recStat, 0),
            'clkReset': get_bytes_from_flag(recStat, 1),
        }
        self.receiving_stamp.week = week
        self.receiving_stamp.TOW = round(rcvTow)
        for i in range(numMeas):
            prMes, cpMes, doMes, gnssId, svId, _, freqId, locktime, cno, prStedv, cpStedv, doStedv, trkStat, _ = \
                struct.unpack('<ddfBBBBHBssssB', msg[32 * i: 32 * (i + 1)])
            trkStat = flag_to_int(trkStat)
            self.satellites.append({
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
            } | get_stamps(self, svId, GNSS(gnssId)))


class NAV_SAT(UbxMessage):
    format = '<LBBBB'
    header = (0x01, 0x35)

    data: dict = {}
    satellites: list = []

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        self.data: dict = {}
        self.satellites: list = []
        iTOW, version, numsSvs, *reversed1 = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.data: dict = {
            'iTOW': iTOW
        }
        for i in range(numsSvs):
            gnssId, svId, cno, elev, azim, prRes, flags = struct.unpack('<BBBbhh4s', msg[12 * i: 12 * (i + 1)])
            flags = flag_to_int(flags)
            self.satellites.append({
                'cno': cno,
                'elev': elev,
                'azim': azim,
                'prRes': prRes * 0.1,
                'qualityInd': get_bytes_from_flag(flags, 0, 1, 2),
                'svUsed': get_bytes_from_flag(flags, 3),
                'health': get_bytes_from_flag(flags, 4, 5),
                'diffCorr': get_bytes_from_flag(flags, 6),
                'smoothed': get_bytes_from_flag(flags, 7),
                'orbitSourse': get_bytes_from_flag(flags, 8, 9, 10),
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
            } | get_stamps(self, svId, GNSS(gnssId)))
        return

class NAV_ORB(UbxMessage):
    format = '<LBBBB'
    header = (0x01, 0x34)

    data: dict = {}
    satellites: list = []

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        self.data: dict = {}
        self.satellites: list = []
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
            self.satellites.append({
                'health': svFlag % 4,
                'visibility': (svFlag // 4) % 4,
                'ephUsability': eph % 32,
                'ephSource': eph // 32,
                'almUsability': alm % 32,
                'almSource': alm // 32,
                'anoAopUsability': otherOrb % 32,
                'type': otherOrb // 32,
            } | get_stamps(self, svId, GNSS(gnssId)))


class RXM_SVSI(UbxMessage):
    format = '<LhBB'
    header = (0x02, 0x20)

    data: dict = {}
    satellites: list = []

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        self.data: dict = {}
        self.satellites: list = []
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
            #TODO: svId -> svId + gnssId - прописать соответствие
            self.satellites.append({
                'azim': azim,
                'elev': elev,
                'ura': get_bytes_from_flag(sv_flag, 0, 1, 2, 3),
                'healthy': get_bytes_from_flag(sv_flag, 4),
                'ephVal': get_bytes_from_flag(sv_flag, 5),
                'almVal': get_bytes_from_flag(sv_flag, 6),
                'notAvail': get_bytes_from_flag(sv_flag, 7),
                'almAge': (age_flag & 0x0f) - 4,
                'ephAge': ((age_flag & 0xf0) >> 4) - 4,
            } | get_stamps(self, svId, GNSS.GPS))


class RXM_MEASX(UbxMessage):
    format = '<B3sLLL4sLHHH2sHBs8s'
    header = (0x02, 0x14)

    data: dict = {}
    satellites: list = []

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        self.data: dict = {}
        self.satellites: list = []
        version, _, gpsTOW, gloTOW, bdsTOW, _, qzssTOW, gpsTOWacc, gloTOWacc, bdsTOWacc, _, qzssTOWacc, numSV, flags, _ \
            = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        flags = flag_to_int(flags)
        self.receiving_stamp.TOW = round(gpsTOW / 1000)
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
            self.satellites.append({
                # 'cNo': cno, # carrier noise ratio (0..63) - коэффициент -> не то
                'mpathIndic': mpathIndic,
                'dopplerMS': dopplerMS * 0.04,
                'dopplerHz': dopplerHz * 0.2,
                'wholeChips': wholeChips,
                'fracChips': fracChips,
                'codePhase': codePhase * 2 ** (-21),
                'intCodePhase': intCodePhase,
                'pseuRangeRMSErr': pseuRangeRMSErr, # индекс
                'prRMSer': calc_prRMSer(pseuRangeRMSErr), # метры
            } | get_stamps(self, svId, GNSS(gnssId)))


class RXM_SFRBX(UbxMessage):
    format = '<BBBBBBBB'
    header = (0x02, 0x13)
    def __init__(self, msg: bytes, receiving_stamp: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_stamp)
        gnssID, svId, _, freqId, numWords, chn, version, _ = \
            struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        if version == 0x02:
            chn = None
        msg = msg[struct.calcsize(self.format):]
        self.data = {
            'gnssId': GNSS(gnssID),
            'svId': svId,
            'freqId': freqId,
            'chn': chn,
        }
        if gnssID == 0:
            self.signal = GPSSingalsParser.parse(GPSSingalsParser.gps_join_sf(msg))
            if 'name' in self.signal.keys():
                self.data['id'] = -1 # общие данные группировки
            elif 'SV_ID' in self.signal.keys():
                self.data['id'] = self.signal['SV_ID'] # данные конкретного спутника
            else:
                self.data['id'] = 0 # данные спутника, с которого получен сигнал
        if gnssID == 6:
            id, StringN, superframeN, update_data = GLONASSSignalsParser.parse(msg)
            self.signal = update_data
            self.signal |= { f'sfN{StringN}': superframeN }
            self.data['id'] = id # 0 -> данные для текущего спутника (или общие), иначе - id = svId полученных данных
            self.data['StringN'] = StringN
            self.data['superframeN'] = superframeN


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
            'week': week,
            'leapS': leapS,
            'tAcc': tAcc,
            'towValid': get_bytes_from_flag(flags, 0),
            'weekValid': get_bytes_from_flag(flags, 1),
            'leapSValid': get_bytes_from_flag(flags, 2),
        }


class NAV_CLOCK(UbxMessage):
    format = '<LllLL'
    header = (0x01, 0x22)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, clkB, clkD, tAcc, fAcc = struct.unpack(self.format, msg)
        self.data = {
            'iTOW': iTOW,
            'clkB': clkB,
            'clkD': clkD,
            'tAcc': tAcc,
            'fAcc': fAcc,
        }


class NAV_DOP(UbxMessage):
    format = '<LHHHHHHH'
    header = (0x01, 0x04)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP  = struct.unpack(self.format, msg)
        self.data = {
            'iTOW': iTOW,
            'gDOP': gDOP * 0.01,
            'pDOP': pDOP * 0.01,
            'tDOP': tDOP * 0.01,
            'vDOP': vDOP * 0.01,
            'hDOP': hDOP * 0.01,
            'nDOP': nDOP * 0.01,
            'eDOP': eDOP * 0.01,
        }
        a=0

class NAV_POSECEF(UbxMessage):
    format = '<LlllL'
    header = (0x01, 0x01)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        iTOW, ecefX, ecefY, ecefZ, pAcc = struct.unpack(self.format, msg)
        self.data = {
            'iTOW': iTOW,
            'ecefX': ecefX * 1e-2,
            'ecefY': ecefY * 1e-2,
            'ecefZ': ecefZ * 1e-2,
            'pAcc': pAcc * 1e-2,
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
        self.stamp = {'svId': svId, 'gnssId': GNSS.GPS}
        if HOW == 0:
            self.data = None
            return
        data1 = GPSSingalsParser.parse_aid(GPSSingalsParser.gps_join_sf(msg[:32], 'AID'), 1)
        data2 = GPSSingalsParser.parse_aid(GPSSingalsParser.gps_join_sf(msg[32:64], 'AID'), 2)
        data3 = GPSSingalsParser.parse_aid(GPSSingalsParser.gps_join_sf(msg[64:], 'AID'), 3)
        self.data = data1 | data2 | data3


class AID_ALM(UbxMessage):
    format = '<LL'
    header = (0x0B, 0x30)

    def __init__(self, msg: bytes, receiving_TOW: int or datetime = BASE_TIME_STAMP()):
        super().__init__(receiving_TOW)
        svId, week = struct.unpack(self.format, msg[:struct.calcsize(self.format)])
        msg = msg[struct.calcsize(self.format):]
        self.stamp = {'svId': svId, 'gnssId': GNSS.GPS}
        if week == 0:
            self.data = None
            return

        data = GPSSingalsParser.parse_aid(GPSSingalsParser.gps_join_sf(msg, 'AID'), 5)
        self.data = data | {'week': week}




