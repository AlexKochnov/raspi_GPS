from enum import IntFlag, Enum


class FormatCoords(IntFlag):
    XYZ = 0x01 << 14
    LLA = 0x10 << 14
    ABC = 0x100 << 14

    def __or__(self, other):
        return FormatCoords(self.value | other.value)


class GNSS(Enum):
    GPS = 0
    SBAS = 1
    Galileo = 2
    BeiDou = 3
    IMEA = 4
    QZSS = 5
    GLONASS = 6
    default = -1

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