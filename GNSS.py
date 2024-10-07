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
    gnssId: GNSS
    dataType: NavDataType

    def __init__(self, gnssId, dataType):
        self.gnssId = gnssId
        self.dataType = dataType

    def __str__(self):
        return f'<Source: {self.gnssId.name}/{self.dataType.name}>'

    def __repr__(self):
        return str(self)

    def __eq__(self, other):
        return self.gnssId == other.gnssId and self.dataType == other.dataType

    def __hash__(self):
        return hash(self.gnssId.value + self.dataType.value)

    @staticmethod
    def multi_get(gnss_s: list[GNSS], multi_gnss_task=False):
        result = []
        result.append(Source(GNSS.receiver, NavDataType.receiver))
        if multi_gnss_task:
            gnss_s.append(GNSS.ALL)
        for gnss in gnss_s:
            result.append(Source(gnss, NavDataType.ALM))
            result.append(Source(gnss, NavDataType.EPH))
        return result

ReceiverSource = Source(GNSS.receiver, NavDataType.receiver)
