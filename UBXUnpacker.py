import struct
from datetime import datetime
from enum import Enum
from math import sqrt, atan, atan2, pi

from tabulate import tabulate


# для отладки
# from tabulate import tabulate


class MessageTypes(Enum):
    EPH = 1
    ALM = 2
    NAV = 3
    RXM = 4
    # NAV_TIMEGPS = 5


class GNSS(Enum):
    GPS = 0
    SBAS = 1
    Galileo = 2
    BeiDou = 3
    IMEA = 4
    QZSS = 5
    GLONASS = 6


class Struct(object):
    def __init__(self, *args, **kwargs):
        self.update(**kwargs)

    def __add__(self, other):
        res = Struct(**self.__dict__)
        res.update(**other.__dict__)
        return res

    # def __sub__(self, other):
    #     res = Struct({k: v for k, v in self.__dict__.items() if k not in other})
    def filter(self, *args):
        return Struct(**{k: v for k, v in self.__dict__.items() if k not in args})

    def __iadd__(self, other):
        self.update(**other.__dict__)
        return self

    def update(self, *args, **kwargs):
        self.__dict__.update(**kwargs)

    def __str__(self):
        return str(self.__dict__)

    def __repr__(self):
        return str(self.__dict__)

    # def short_str(self) -> str:
    #     return "\n".join(f'{k}: {v}' if v is not list else f'{k}: [{len(v)}: {v[0]}, {v[1]}, ...'
    #                      for k, v in self.__dict__.items())

    def values(self):
        return self.__dict__.values()

    def keys(self):
        return self.__dict__.keys()

    def items(self):
        return self.__dict__.items()


class Message:
    receiving_time: datetime = None
    header: str = None

    def __init__(self, receiving_time: datetime = None):
        self.receiving_time = receiving_time or datetime.now()


class Satellite:
    def __init__(self, gnssId, svId):
        self.gnssId = GNSS(gnssId)
        self.svId = svId


def flag_to_int(flags: bytes) -> int:
    # return sum([(flags[i] & 0xff) << 8 * (len(flags) - 1 - i) for i in range(len(flags))])
    return sum(flags[i] << 8 * i for i in range(len(flags)))
    pass


def get_bytes_from_flag(flags: int, *pattern) -> int:
    return (flags >> min(pattern)) & sum(1 << (x - min(pattern)) for x in pattern)


def get_all_flags(flags: bytes, patterns: list[list[int] or int]) -> list[int]:
    flags = flag_to_int(flags)
    return [get_bytes_from_flag(flags, pat) for pat in patterns]


def cut_msg(length, byte_str: bytes) -> (bytes, bytes):
    return byte_str[:length], byte_str[length:]


def parse_message(msg: bytes) -> list[bytes]:
    """
    :param msg:
    :return:
    msg_useless = msg[6:-2]
    msg_header = msg[2:]
    msg_class = msg[2]
    msg_ID = msg[3]
    msg_len_byte = msg[4:6]
    msg_ck =msg[-2:]
    """
    return [msg[6:-2], msg[2:], msg[2], msg[3], msg[4:6], msg[-2:]]


def ecef2lla(x, y, z):
    # x, y and z are scalars or vectors in meters

    a = 6378137
    a_sq = a ** 2
    e = 8.181919084261345e-2
    e_sq = 6.69437999014e-3

    f = 1 / 298.257223563
    b = a * (1 - f)

    # calculations:
    r = sqrt(x ** 2 + y ** 2)
    ep_sq = (a ** 2 - b ** 2) / b ** 2
    ee = (a ** 2 - b ** 2)
    f = (54 * b ** 2) * (z ** 2)
    g = r ** 2 + (1 - e_sq) * (z ** 2) - e_sq * ee * 2
    c = (e_sq ** 2) * f * r ** 2 / (g ** 3)
    s = (1 + c + sqrt(c ** 2 + 2 * c)) ** (1 / 3.)
    p = f / (3. * (g ** 2) * (s + (1. / s) + 1) ** 2)
    q = sqrt(1 + 2 * p * e_sq ** 2)
    r_0 = -(p * e_sq * r) / (1 + q) + sqrt(
        0.5 * (a ** 2) * (1 + (1. / q)) - p * (z ** 2) * (1 - e_sq) / (q * (1 + q)) - 0.5 * p * (r ** 2))
    u = sqrt((r - e_sq * r_0) ** 2 + z ** 2)
    v = sqrt((r - e_sq * r_0) ** 2 + (1 - e_sq) * z ** 2)
    z_0 = (b ** 2) * z / (a * v)
    h = u * (1 - b ** 2 / (a * v))
    phi = atan((z + ep_sq * z_0) / r)
    lambd = atan2(y, x)

    return phi * 180 / pi, lambd * 180 / pi, h


def read_sf(msg: bytes) -> (str, bytes):
    sf = struct.unpack('4s' * 8, msg[:32])
    res = '0' * 60
    for word in sf:
        word = bytes([word[2], word[1], word[0], word[3]])
        res += f'{int(word.hex(), 16):032b}'[:-2]
    return res, msg[32:]


def bin2dec(binaryStr: str) -> int:
    return int(binaryStr, 2)


def twosComp2dec(binaryStr: str) -> int:
    # функция с проверкой знака
    intNumber = int(binaryStr, 2)
    if binaryStr[0] == '1':
        intNumber -= 2 ** len(binaryStr)
    return intNumber


class NAV_SAT(Message):
    header = '<LBBBB'
    SAT = dict()

    class SV_SAT:
        def __init__(self, cno, elev, azim, prRes, flags):
            self.cno = cno
            self.elev = elev
            self.azim = azim
            self.prRes = prRes
            int_flags = flag_to_int(flags)
            self.flags = {
                'ualityInd': get_bytes_from_flag(int_flags, 0, 1, 2),
                'svUsed': get_bytes_from_flag(int_flags, 3),
                'health': get_bytes_from_flag(int_flags, 4, 5),
                'ephAvail': get_bytes_from_flag(int_flags, 11),
                'almAvail': get_bytes_from_flag(int_flags, 12)
            }

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, version, numsSvs, *reversed1 = struct.unpack(self.header, msg[:struct.calcsize(self.header)])
        msg = msg[struct.calcsize(self.header):]
        for i in range(numsSvs):
            gnssId, svId, cno, elev, azim, prRes, flags = struct.unpack('<BBBbhh4s', msg[12 * i: 12 * (i + 1)])
            self.SAT[Satellite(gnssId, svId)] = self.SV_SAT(cno, elev, azim, prRes, flags)


class NAV_ORB(Message):
    header = '<LBBBB'
    ORB = dict()

    class SV_ORB:
        def __init__(self, svFlag, eph, alm, otherOrb):
            svFlag = flag_to_int(svFlag)
            eph = flag_to_int(eph)
            alm = flag_to_int(alm)
            otherOrb = flag_to_int(otherOrb)

            self.health = svFlag % 4
            self.visibility = (svFlag // 4) % 4,
            self.ephUsability = eph % 32
            self.ephSource = eph // 32,
            self.almUsability = alm % 32
            self.almSource = alm // 32,
            self.anoAopUsability = otherOrb % 32
            self.type = otherOrb // 32

    def __init__(self, msg: bytes, receiving_time: datetime = None):
        super().__init__(receiving_time)
        self.iTOW, version, numsSvs, *reversed1 = struct.unpack(self.header, msg[:struct.calcsize(self.header)])
        msg = msg[struct.calcsize(self.header):]
        for i in range(numsSvs):
            gnssId, svId, svFlag, eph, alm, otherOrb = struct.unpack('<BBssss', msg[6 * i: 6 * (i + 1)])
            self.ORB[Satellite(gnssId, svId)] = self.SV_ORB(svFlag, eph, alm, otherOrb)


def NAV_TIMEGPS(msg: bytes, receiving_time: datetime or None = None) -> (MessageTypes, list):
    # The precise GPS time of week in seconds is: (iTOW * 1e-3) + (fTOW * 1e-9)
    if receiving_time is None:
        receiving_time = datetime.now()
    iTOW, fTOW, week, leapS, valid_flag, tAcc = struct.unpack('<LlhbsL', msg)
    flags = flag_to_int(valid_flag)
    bitfield = lambda *args: get_bytes_from_flag(flags, args)
    data = Struct(type='NAV_TIMEGPS', sys_time=receiving_time, iTOW=iTOW, fTOW=fTOW, TOW=iTOW * 1e-3 + fTOW * 1e-9,
                  week=week, leapS=leapS, tAcc_ftow=tAcc,
                  valid=Struct(tow=bitfield(0), week=bitfield(1), leapS=bitfield(1)))
    return MessageTypes.NAV, data


def AID_EPH(msg: bytes, receiving_time: datetime or None = None) -> (MessageTypes, list):
    if receiving_time is None:
        receiving_time = datetime.now()
    header = '<LL'
    SV_ID, HOW = struct.unpack(header, msg[:struct.calcsize(header)])
    msg = msg[struct.calcsize(header):]
    if HOW == 0:
        return None

    ### SubFrame 1
    subframe, msg = read_sf(msg)
    week = bin2dec(subframe[60:70]) + 0 * 1024 * 2
    accuracy = bin2dec(subframe[72:76])
    health = bin2dec(subframe[76:82])
    IODC = bin2dec(subframe[82:84] + subframe[196:204])
    Tgd = twosComp2dec(subframe[195:204]) * 2 ** (- 31)
    Toc = bin2dec(subframe[218:234]) * 2 ** 4
    af2 = twosComp2dec(subframe[240:248]) * 2 ** (- 55)
    af1 = twosComp2dec(subframe[248:264]) * 2 ** (- 43)
    af0 = twosComp2dec(subframe[270:292]) * 2 ** (- 31)

    ### Subframe 2
    subframe, msg = read_sf(msg)
    IODE1 = bin2dec(subframe[60:68])
    Crs = twosComp2dec(subframe[68:84]) * 2 ** (- 5)
    dn = twosComp2dec(subframe[90:106]) * 2 ** (- 43)
    M0 = twosComp2dec(subframe[106:114] + subframe[120:144]) * 2 ** (- 31)
    Cuc = twosComp2dec(subframe[150:166]) * 2 ** (- 29)
    e = bin2dec(subframe[166:174] + subframe[180:204]) * 2 ** (- 33)
    Cus = twosComp2dec(subframe[210:226]) * 2 ** (- 29)
    sqrtA = bin2dec(subframe[226:234] + subframe[240:264]) * 2 ** (- 19)
    Toe = bin2dec(subframe[270:286]) * 2 ** 4

    ### Subframe 3
    subframe, msg = read_sf(msg)
    Cic = twosComp2dec(subframe[60:76]) * 2 ** (- 29)
    W0 = twosComp2dec(subframe[76:84] + subframe[90:114]) * 2 ** (- 31)
    Cis = twosComp2dec(subframe[120:136]) * 2 ** (- 29)
    i0 = twosComp2dec(subframe[136:144] + subframe[150:174]) * 2 ** (- 31)
    Crc = twosComp2dec(subframe[180:196]) * 2 ** (- 5)
    w = twosComp2dec(subframe[196:204] + subframe[210:234]) * 2 ** (- 31)
    Wdot = twosComp2dec(subframe[240:264]) * 2 ** (- 43)
    IODE2 = bin2dec(subframe[270:278])
    IDOT = twosComp2dec(subframe[278:292]) * 2 ** (- 43)

    EPH = [SV_ID, week, Toe, Toc, IODE1, IODE2, IODC, IDOT, Wdot, Crs, Crc, Cus, Cuc, Cis, Cic, dn, i0, e, sqrtA, M0,
           W0, w, Tgd, af2, af1, af0, health, accuracy, receiving_time]

    return MessageTypes.EPH, EPH


def AID_ALM(msg: bytes, receiving_time: datetime or None = None) -> (MessageTypes, list):
    if receiving_time is None:
        receiving_time = datetime.now()
    header = '<LL'
    svid, week = struct.unpack(header, msg[:struct.calcsize(header)])
    msg = msg[struct.calcsize(header):]
    if week == 0:
        return None

    subframe, msg = read_sf(msg)
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

    ALM = [SV_ID, week, Toa, e, delta_i, Wdot, sqrtA, W0, w, M0, af0, af1, health, Data_ID, receiving_time]

    return MessageTypes.ALM, ALM


def ParseSfGPS(msg: bytes, numWords):
    sf = struct.unpack('4s' * numWords, msg[:40])
    res = ''
    for word in sf:
        # word = bytes([word[2], word[1], word[0], word[3]])
        res += f'{int(word.hex(), 16):032b}'[2:]
    return res


class GPSParser:

    def parse(self, subframe, subframe_number: int) -> list:
        return {
            1: self.ParseSf1,
            2: self.ParseSf2,
            3: self.ParseSf3,
            5: self.ParseSf5,
        }[subframe_number](subframe)

    class Sf1:
        __slots__ = ['week', 'accuracy', 'health', 'IODC', 'Tgd', 'Toc', 'af2', 'af1', 'af0']

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


def RXM_SFRBX(msg: bytes, receiving_time: datetime or None = None) -> dict[str: object]:
    if receiving_time is None:
        receiving_time = datetime.now()
    header = '<BBBBBBBB'
    gnssID, svId, _, freqId, numWords, chn, version, _ = struct.unpack(header, msg[:struct.calcsize(header)])
    if version == 0x02:
        chn = None
    msg = msg[struct.calcsize(header):]
    if gnssID == GNSS.GPS.value:
        subframe = ParseSfGPS(msg, numWords)
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
    print(f'SFRBX : {gnssID} : {svId} : {sfId} : {subframe}')
    print('\n')
    a = 0

    data = Struct(type='RXM_SVSI', sys_time=receiving_time, gnssID=gnssID, svId=svId)
    data.plb = msg
    return data

    # for i in range(numSV):
    #     svId, sv_flag, azim, elev, age_flag = struct.unpack('<Bshbs', msg[+ i * 6: 6 * (i + 1)])
    #     SatData = Struct(svId=svId, azim=azim, elev=elev, sv_flag=bin(ord(sv_flag)), age_flag=bin(ord(age_flag)))
    #     data.SatSVSI.append(SatData)
    #
    # return MessageTypes.RXM, data


def NAV_POSECEF(msg: bytes, receiving_time: datetime or None = None) -> dict[str: object]:
    if receiving_time is None:
        receiving_time = datetime.now()
    iTOW, ecefX, ecefY, ecefZ, tAcc = struct.unpack('<LlllL', msg)
    data = Struct(type='NAV_POSECEF', sys_time=receiving_time, iTOW=iTOW, tAcc_ecefP=tAcc / 100,
                  ecefX=ecefX / 100.0, ecefY=ecefY / 100.0, ecefZ=ecefZ / 100.0)
    data.ECEF = (data.ecefX, data.ecefY, data.ecefZ)
    data.LLA = ecef2lla(*data.ECEF)
    return MessageTypes.NAV, data


def NAV_VELECEF(msg: bytes, receiving_time: datetime or None = None) -> dict[str: object]:
    if receiving_time is None:
        receiving_time = datetime.now()
    iTOW, ecefVX, ecefVY, ecefVZ, tAcc = struct.unpack('<LlllL', msg)
    data = Struct(type='NAV_VELECEF', sys_time=receiving_time, iTOW=iTOW, tAcc_ecefV=tAcc / 100,
                  ecefVX=ecefVX / 100.0, ecefVY=ecefVY / 100.0, ecefVZ=ecefVZ / 100.0)
    data.ECEFV = (data.ecefVX, data.ecefVY, data.ecefVZ)
    return MessageTypes.NAV, data


def RXM_SVSI(msg: bytes, receiving_time: datetime or None = None) -> dict[str: object]:
    if receiving_time is None:
        receiving_time = datetime.now()
    header = '<LhBB'
    iTOW, week, numVis, numSV = struct.unpack(header, msg[:struct.calcsize(header)])
    data = Struct(type='RXM_SVSI', sys_time=receiving_time, iTOW=iTOW, SatSVSI=[])
    msg = msg[struct.calcsize(header):]

    for i in range(numSV):
        svId, sv_flag, azim, elev, age_flag = struct.unpack('<Bshbs', msg[+ i * 6: 6 * (i + 1)])
        SatData = Struct(svId=svId, azim=azim, elev=elev, sv_flag=bin(ord(sv_flag)), age_flag=bin(ord(age_flag)))
        data.SatSVSI.append(SatData)

    return MessageTypes.RXM, data
