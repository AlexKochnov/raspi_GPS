# from UBXUnpacker import *
# from pyubx2 import UBXMessage, SET
import struct



def calc_checksum(cmd: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    for c in cmd:
        ck_a = ck_a + c
        ck_b = ck_b + ck_a
    return struct.pack('B', ck_a & 0xff) + struct.pack('B', ck_b & 0xff)


def check_cks(cmd: bytes) -> bool:
    return calc_checksum(cmd[2:-2]) == cmd[-2:]




class POOLMessages:
    EPH = b'\x0B\x31' + b'\x00\x00'  # AID-EPH
    ALM = b'\x0B\x30' + b'\x00\x00'  # AID-ALM
    RST = b'\x06\x04\x04\x00\xFF\xFF\x00\x00'  # CFG-RST
    RAWX = b'\x02\x15' + b'\x00\x00'
    RATE_GET = b'\x06\x08' + b'\x00\x00'
    RATE_SET = b'\x06\x08' + b'\x06\x00' + b'\x00\x04' + b'\x01\x00' + b'\x00\x00'

    MON_GNSS = b'\x0A\x28' + b'\x00\x00'
    GNSS_check = b'\x06\x3E' + b'\x00\x00' # cfg gnss get
    GLO = b'\x06\x3E' + b'\x0C\x00' + b'\x00\x20\x20\x02' + b'\x06\x20\x20\x00' + b'\x00\x01\x00\x00'  # Glonas
    ON_ALL = b'\x06\x3E' + b'\x0C\x00' + b'\x00\x20\x20\x02' + \
             b'\x06\x20\x20\x00' + b'\x00\x10\x00\x00' + \
             b'\x01\x20\x20\x00' + b'\x00\x01\x00\x00' + \
             b'\x00\x20\x20\x00' + b'\x00\x10\x00\x01'  # GPS

    CFG_PRT_GET = b'\x06\x00' + b'\x00\x00'

    @staticmethod
    def CFG_PRT(port: int):
        return b'\x06\x00' + b'\x01\x00' + struct.pack('B', port)


MSG2pool = [
    # b'\x06\x04\x04\x00\xFF\xFF\x00\x00' # CFG-RST
    b'\x0B\x30' + b'\x00\x00',  # aid-alm
    b'\x0B\x31' + b'\x00\x00',  # aid-eph
    # b'\x0B\x33' + b'\x00\x00',  # aid-aop

    # b'\x02\x14' + b'\x00\x00' # rxm-measx
    # b'\x02\x15' + b'\x00\x00'  # rxm-rawx pooling
    # b'\x02\x13' + b'\x00\x00' # rxm-sfrbx

    # b'\x06\x04\x04\x00\x00\x00\x00\x00' # CFG-RST - reset with Cold start and Hardware reset immediately
    # b'\xb5b\x06\x04\x04\x00\x00\x00\x00\x00\x0ed'

    # b'\x09\x14\x08\x00\x01\x00\x00\x00' ##sos for reset - not work

    # b'\x0A\x32' + b'\x00\x00', # MON-BATCH
    # b'\x0A\x28' + b'\x00\x00', # MON-GNSS
    # b'\x0A\x2E' + b'\x00\x00' # MON-SMGR

    # b'\x0D\x04' + b'\x00\x00' # TIM-SVIN

    # b'\x10\x14' + b'\x00\x00' # ESF-ALG
    # b'\x10\x10' + b'\x00\x00' # ESF-STATUS

    # b'\x13\x06' + b'\x00\x00' #+ b'\x02\x00' # MGA-GLO-ALM

    # b'\x06\x3E' +b'\x0C\x00' + b'\x00\x20\x20\x01' + b'\x06\x20\x20\x00' + b'\x00\x01\x00\x01' # GLONAS-1
    # b'\x06\x3E' + b'\x0C\x00' + b'\x00\x20\x20\x02' +
    #     b'\x06\x20\x20\x00' + b'\x00\x10\x00\x00' +  # Glonas
    #     b'\x01\x20\x20\x00' + b'\x00\x01\x00\x00' + # SBAS
    #     b'\x00\x20\x20\x00' + b'\x00\x10\x00\x01'  # GPS
    #
]


def set_rate(msgClass: hex, msgID: hex, rateUART1: int) -> bytes:
    cmd = b'\x06\x01' + b'\x03\x00' + msgClass.to_bytes() + msgID.to_bytes() + rateUART1.to_bytes()
    return b'\xb5b' + cmd + calc_checksum(cmd)
    # return UBXMessage('CFG', 'CFG-MSG', SET, msgClass=msgClass, msgID=msgID, rateUART1=rateUART1).serialize()


def check_rate(msgClass: hex, msgID: hex) -> bytes:
    cmd = b'\x06\x01' + b'\x02\x00' + msgClass.to_bytes() + msgID.to_bytes()
    return b'\xb5b' + cmd + calc_checksum(cmd)

def cmd2msg(cmd: bytes):
    return b'\xb5b' + cmd + calc_checksum(cmd)

# CFG_PRT = b'\x06\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00' + struct.pack('I', 115200) + b'\x00\x00\x00\x00\x00\x00\x00\x00'



MSG2set = [

    # set_rate(msgClass=0x02, msgID=0x13, rateUART1=1),  # RXM-SFRBX

    set_rate(msgClass=0xF0, msgID=0x00, rateUART1=0),  # GGA
    set_rate(msgClass=0xF0, msgID=0x01, rateUART1=0),  # GLL
    set_rate(msgClass=0xF0, msgID=0x02, rateUART1=0),  # GSA
    set_rate(msgClass=0xF0, msgID=0x03, rateUART1=0),  # GSV
    set_rate(msgClass=0xF0, msgID=0x04, rateUART1=1),  # RMC
    set_rate(msgClass=0xF0, msgID=0x05, rateUART1=0),  # VTG
    set_rate(msgClass=0xF0, msgID=0x08, rateUART1=0),  # ZDA
    set_rate(msgClass=0xF0, msgID=0x41, rateUART1=0),  # TXT
    set_rate(msgClass=0xF0, msgID=0x0D, rateUART1=0),  # GNS

    set_rate(msgClass=0xF1, msgID=0x03, rateUART1=1),  # PUBX SVSTATUS

    # set_rate(msgClass=0xF0, msgID=0x00, rateUART1=1),  # GGA
    # set_rate(msgClass=0xF0, msgID=0x02, rateUART1=1),  # GSA
    # set_rate(msgClass=0xF0, msgID=0x03, rateUART1=1),  # GSV
    # set_rate(msgClass=0xF0, msgID=0x04, rateUART1=1),  # RMC
    # set_rate(msgClass=0xF0, msgID=0x05, rateUART1=1),  # VTG

    # set_rate(msgClass=0x01, msgID=0x01, rateUART1=1),  # NAV-POSECEF
    # set_rate(msgClass=0x01, msgID=0x11, rateUART1=1),  # NAV-VELECEF
    set_rate(msgClass=0x01, msgID=0x20, rateUART1=1),  # NAV-TIMEGPS
    set_rate(msgClass=0x01, msgID=0x34, rateUART1=1),  # NAV-ORB
    set_rate(msgClass=0x01, msgID=0x35, rateUART1=1),  # NAV-SAT
    set_rate(msgClass=0x01, msgID=0x31, rateUART1=1),  # NAV-DGPS
    set_rate(msgClass=0x01, msgID=0x30, rateUART1=1),  # NAV-SVINFO

    set_rate(msgClass=0x02, msgID=0x15, rateUART1=1),  # RXM-RAWX
    set_rate(msgClass=0x02, msgID=0x20, rateUART1=1),  # RXM-SVSI
    set_rate(msgClass=0x02, msgID=0x14, rateUART1=1),  # RXM-MEASX

    set_rate(msgClass=0x01, msgID=0x01, rateUART1=0),  # NAV-POSECEF
    set_rate(msgClass=0x01, msgID=0x11, rateUART1=0),  # NAV-VELECEF
    # set_rate(msgClass=0x01, msgID=0x20, rateUART1=0),  # NAV-TIMEGPS
    # set_rate(msgClass=0x01, msgID=0x34, rateUART1=0),  # NAV-ORB
    # set_rate(msgClass=0x01, msgID=0x35, rateUART1=0),  # NAV-SAT

    # # #
    # set_rate(msgClass=0x02, msgID=0x15, rateUART1=1),  # RXM-RAWX
    set_rate(msgClass=0x02, msgID=0x13, rateUART1=0),  # RXM-SFRBX
    # set_rate(msgClass=0x02, msgID=0x20, rateUART1=0),  # RXM-SVSI
    set_rate(msgClass=0x02, msgID=0x61, rateUART1=0),  # RXM-IMES


    set_rate(msgClass=0x0A, msgID=0x07, rateUART1=0),  # mon rxbuf
    set_rate(msgClass=0x0A, msgID=0x08, rateUART1=1),  # mon txbuf
    set_rate(msgClass=0x0A, msgID=0x32, rateUART1=0),  # mon txbuf

    # check_rate(0x02, 0x15)



    # set_rate(msgClass=0x10, msgID=0x14, rateUART1=1),  # ESF-ALG
    # set_rate(msgClass=0x10, msgID=0x10, rateUART1=1),  # ESF-STATUS

    # set_rate(msgClass=0x02, msgID=0x13, rateUART1=1),
    # set_rate(msgClass=0x13, msgID=0x06, rateUART1=1), # MGA-GLO-ALM

]


def flag_to_int(flags: bytes) -> int:
    # return sum([(flags[i] & 0xff) << 8 * (len(flags) - 1 - i) for i in range(len(flags))])
    return sum(flags[i] << 8 * i for i in range(len(flags)))
    pass


def get_bytes_from_flag(flags: int, *pattern) -> int:
    return (flags >> min(pattern)) & sum(1 << (x - min(pattern)) for x in pattern)
