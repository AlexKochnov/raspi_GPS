from enum import Enum

import UBXMessages
from UBXMessages import *
from NMEAMessages import *


class POOLMessages:
    EPH = b'\x0B\x31' + b'\x00\x00'  # AID-EPH
    ALM = b'\x0B\x30' + b'\x00\x00'  # AID-ALM
    RST = b'\x06\x04\x04\x00\xFF\xFF\x00\x00'  # CFG-RST
    RAWX = b'\x02\x15' + b'\x00\x00'
    RATE_GET = b'\x06\x08' + b'\x00\x00'
    RATE_SET = b'\x06\x08' + b'\x06\x00' + b'\x00\x04' + b'\x01\x00' + b'\x00\x00'

    MON_GNSS = b'\x0A\x28' + b'\x00\x00'
    GNSS_check = b'\x06\x3E' + b'\x00\x00'  # cfg gnss get
    GLO = b'\x06\x3E' + b'\x0C\x00' + b'\x00\x20\x20\x02' + b'\x06\x20\x20\x00' + b'\x00\x01\x00\x00'  # Glonas
    ON_ALL = b'\x06\x3E' + b'\x0C\x00' + b'\x00\x20\x20\x02' + \
             b'\x06\x20\x20\x00' + b'\x00\x10\x00\x00' + \
             b'\x01\x20\x20\x00' + b'\x00\x01\x00\x00' + \
             b'\x00\x20\x20\x00' + b'\x00\x10\x00\x01'  # GPS

    CFG_PRT_GET = b'\x06\x00' + b'\x00\x00'

    CFG_ESRC = b'\x06\x60' + b'\x00\x00'
    CFG_GNSS = b'\x06\x3E' + b'\x00\x00'
    TIM_SMEAS = b'\x0D\x13' + b'\x00\x00'

    @staticmethod
    def CFG_PRT(port: int):
        return b'\x06\x00' + b'\x01\x00' + struct.pack('B', port)


pool_messages = [POOLMessages.EPH, POOLMessages.ALM, ]


def set_rate(msgClass: hex, msgID: hex, rateUART1: int) -> bytes:
    cmd = b'\x06\x01' + b'\x03\x00' + msgClass.to_bytes() + msgID.to_bytes() + rateUART1.to_bytes()
    return b'\xb5b' + cmd + calc_ubx_checksum(cmd)
    # return UBXMessage('CFG', 'CFG-MSG', SET, msgClass=msgClass, msgID=msgID, rateUART1=rateUART1).serialize()


def ON(id):
    return set_rate(id.value[0], id.value[1], 1)


def OFF(id):
    return set_rate(id.value[0], id.value[1], 0)


def check_rate(msgClass: hex, msgID: hex) -> bytes:
    cmd = b'\x06\x01' + b'\x02\x00' + msgClass.to_bytes() + msgID.to_bytes()
    return b'\xb5b' + cmd + calc_ubx_checksum(cmd)


class MSG(Enum):
    NMEA_GGA = 0xF0, 0x00
    NMEA_GLL = 0xF0, 0x01
    NMEA_GSA = 0xF0, 0x02
    NMEA_GSV = 0xF0, 0x03
    NMEA_RMC = 0xF0, 0x04
    NMEA_VTG = 0xF0, 0x05
    NMEA_ZDA = 0xF0, 0x08
    NMEA_TXT = 0xF0, 0x41
    NMEA_GNS = 0xF0, 0x0D

    PUBX_SVSTATUS = 0xF1, 0x03

    NAV_TIMEGPS = UBXMessages.NAV_TIMEGPS.header
    NAV_ORB = UBXMessages.NAV_ORB.header
    NAV_SAT = UBXMessages.NAV_SAT.header
    NAV_DGPS = 0x01, 0x31
    NAV_SVINFO = 0x01, 0x30
    NAV_POSECEF = UBXMessages.NAV_POSECEF.header
    NAV_VELECEF = UBXMessages.NAV_VELECEF.header
    NAV_CLOCK = UBXMessages.NAV_CLOCK.header

    RXM_RAWX = UBXMessages.RXM_RAWX.header
    RXM_SVSI = 0x02, 0x20
    RXM_MEASX = UBXMessages.RXM_MEASX.header
    RXM_SFRBX = UBXMessages.RXM_SFRBX.header

    MON_RXBUF = 0x0A, 0x07
    MON_TXBUF = 0x0A, 0x08


ALL = set(MSG)
ON_LIST = {MSG.NAV_SAT, MSG.NAV_ORB, MSG.NAV_SVINFO, MSG.NAV_TIMEGPS, MSG.NAV_POSECEF, MSG.NAV_DGPS, MSG.NAV_CLOCK,
           MSG.RXM_RAWX, MSG.RXM_MEASX, MSG.RXM_SFRBX, #MSG.RXM_SVSI,
           MSG.NMEA_RMC,
           MSG.MON_RXBUF, MSG.MON_TXBUF,
           MSG.NMEA_RMC
           }

tune_messages = list(map(ON, ON_LIST)) + list(map(OFF, ALL - ON_LIST))
