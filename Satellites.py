from GPSUtils import *
from GPSUtils import GNSS
from UBXUtils import flag_to_int, get_bytes_from_flag


class SAT:
    def __init__(self, cno, elev, azim, prRes, flags):
        self.cno = cno
        self.elev = elev
        self.azim = azim
        self.prRes = prRes
        flags = flag_to_int(flags)
        self.qualityInd = get_bytes_from_flag(flags, 0, 1, 2)
        self.svUsed = get_bytes_from_flag(flags, 3)
        self.health = get_bytes_from_flag(flags, 4, 5)
        self.ephAvail = get_bytes_from_flag(flags, 11)
        self.almAvail = get_bytes_from_flag(flags, 12)

    def __str__(self):
        return str(self.__dict__)


class ORB:
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

    def __str__(self):
        return str(self.__dict__)


class SVSI:
    def __init__(self, sv_flag, azim, elev, age_flag):
        self.azim = azim
        self.elev = elev
        sv_flag = flag_to_int(sv_flag)
        age_flag = flag_to_int(age_flag)
        self.ura = get_bytes_from_flag(sv_flag, 0, 1, 2, 3)
        self.healthy = get_bytes_from_flag(sv_flag, 4)
        self.ephVal = get_bytes_from_flag(sv_flag, 5)
        self.almVal = get_bytes_from_flag(sv_flag, 6)
        self.notAvail = get_bytes_from_flag(sv_flag, 7)
        self.almAge = (age_flag & 0x0f) - 4
        self.ephAge = ((age_flag & 0xf0) >> 4) - 4

    def __str__(self):
        return str(self.__dict__)


class ALM:
    def __init__(self):
        pass

    def to_list(self):
        pass


class EPH:
    class sf1:
        pass


    class sf2:
        pass

    class sf3:
        pass

    def __init__(self):
        pass

    def to_list(self):
        pass


class Satellite:
    svId: int
    gnssId: GNSS
    sat: SAT = None
    orb: ORB = None
    svsi: SVSI = None
    alm: list
    eph: list

    def __init__(self, gnssId, svId):
        self.gnssId = GNSS(gnssId)
        self.svId = svId
