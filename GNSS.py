from enum import Enum

class GNSS(Enum):
    GPS = 0
    SBAS = 1
    Galileo = 2
    BeiDou = 3
    IMEA = 4
    QZSS = 5
    GLONASS = 6
    default = -1
    receiver = -1
    ALL = 9

    @classmethod
    def _missing_(cls, value):
        return cls.default


GNSSLen = {
    GNSS.GPS: 32,
    GNSS.GLONASS: 24
}


def get_GNSS_len(gnss: GNSS):
    if gnss in GNSSLen.keys():
        return GNSSLen[gnss]
    return 0

class NavDataType(Enum):
    default = 0
    receiver = 0
    ALM = 1000
    EPH = 2000

class Format(Enum):
    XYZ = 10000
    LLA = 20000

class Source:
    def __init__(self, source_score: int):
        self.source = source_score

    @staticmethod
    def get(gnss: GNSS, ndt: NavDataType):
        return Source(gnss.value + ndt.value)

    @staticmethod
    def divide(source):
        return GNSS(source.source % 1000), NavDataType(source.source // 1000)

    @staticmethod
    def multi_get(gnss_s: list[GNSS]):
        result = []
        for gnss in gnss_s:
            result.append(Source.get(gnss, NavDataType.ALM))
            result.append(Source.get(gnss, NavDataType.EPH))
        result.append(Source.get(GNSS.receiver, NavDataType.receiver))

