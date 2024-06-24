import math
from math import sin, cos, tan
from datetime import datetime, timedelta

# from matplotlib import pyplot as plt
from serial import Serial

import NMEAUnpacker
import UBXUnpacker
from GPSStorage import GPSStorage
from UBXUtils import *

import Constants

import numpy as np, pandas as pd
# import matplotlib.pyplot as plt

# TODO: убрать современем
from pynmeagps import NMEAReader, NMEAMessage
from pyubx2 import UBXReader


def calc_nmea_cs(cmd):
    if cmd[0] == '$':
        cmd = cmd[1:]
    if '*' in cmd:
        cmd = cmd.split('*')[0]
    checksum = 0
    for sym in cmd:
        checksum ^= ord(sym)
    return format(checksum, '02X')

def tune_baudRate(port, baudrate, timeout):
    ##          b'\xb5b\x06\x00\x14\x00\x01\x00\x00\x00\xc0\x08\x00\x00\x80%\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00\x92\xb5' ## выдан датчиком
    # CFG_PRT = b'\x06\x00\x14\x00\x01\x00\x00\x00\xc0\x08\x00\x00\x80%\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00'
    # CFG_PRT = b'\x06\x00\x14\x00\x01\x00\x00\x00\xc0\x08\x00\x00' + struct.pack('I',
    #                                                                             9600) + b'\x07\x00\x03\x00\x00\x00\x00\x00'
    # CFG_PRT = b'\x06\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00' + struct.pack('I', 19200) + b'\x00\x00\x00\x00\x00\x00\x00\x00'
    # CFG_PRT = (b'\x06\x00' + b'\x14\x00' + b'\x01' + b'\x00' + b'\x00\x00' + b'\x00\x00\x00\x00' +
    cmd = f'$PUBX,41,1,0007,0003,{baudrate},0'
    cmd = cmd + '*' + calc_nmea_cs(cmd) + '\r\n'
    cmd = cmd.encode('utf-8')

    for bR in [9600]: #[4800, 9600, 19200, 38400, 57600, 115200]:#, 230400, 460800]:
        print(f'try connect and change with baudrate: {bR}')
        with Serial(port, bR, timeout=timeout) as stream:
            stream.write(cmd)
    pass


class GPSReader:
    print_noises = True
    print_parsed = True
    print_raw = False

    raw_logger = 'raw.log'
    parsed_logger = 'parsed.log'

    # TODO: modify to command class
    PoolQ = [
        POOLMessages.EPH,
        POOLMessages.ALM,

        # POOLMessages.RST,

        # POOLMessages.CFG_PRT(1),

        # POOLMessages.GNSS_check,
        # POOLMessages.MON_GNSS,
        # POOLMessages.GLO,
        # POOLMessages.ON_ALL,
    ]

    Pool_step = 200
    Pool_start = 15

    counter = 0
    counetr2 = 0

    def __init__(self, port="/dev/ttyS0", baudrate=115200, timeout=1):  # timeout влияет на буфер, 0.1 мало
        # tune_baudRate(port, baudrate, timeout)
        self.stream = Serial(port, baudrate, timeout=timeout)
        self.tune()

    def __iter__(self):
        while True:
            yield self.next()

    def next(self):
        self.counter += 1
        if self.counter % self.Pool_step == self.Pool_start:
            self.pool_next()
        else:
            return self.read_next_message()

    def read_next_message(self):
        try:
            hdr1 = self.stream.read(1)
            if hdr1 == b'\xb5':
                return self.parse_ubx()
            elif hdr1 == b'$':
                return self.parse_nmea()
            else:
                if self.print_noises:
                    print(hdr1)
        except Exception as e:
            print(e)

    def tune(self):
        for message in MSG2set:
            print(f'\tTune: {message}')
            self.send(message)

    def send(self, message):
        self.stream.write(message)

    def pool_next(self):
        if self.PoolQ:
            # cmd = self.PoolQ.pop()
            cmd = self.PoolQ[self.counetr2 % len(self.PoolQ)]
            if cmd == POOLMessages.RST:
                with open(GPSReader.parsed_logger, 'a') as logger:
                    logger.write('------------------------------RESET------------------------------\n')
            self.counetr2 += 1
            print(f'\t ---------- Pool: {cmd}')
            self.send(b'\xb5b' + cmd + calc_checksum(cmd))

    def read_UBX(self) -> [None] * 6 or [bytes] * 6:
        hdr = b'\xb5' + self.stream.read(1)
        if hdr != b'\xb5b':
            return [None] * 6
        clsid = self.stream.read(1)
        msgid = self.stream.read(1)
        lenb = self.stream.read(2)
        leni, *_ = struct.unpack('H', lenb)
        plb = b''
        while leni > 800:
            plb += self.stream.read(800)
            leni -= 800
        plb += self.stream.read(leni)
        cks = self.stream.read(2)
        return hdr, clsid, msgid, lenb, plb, cks

    def parse_ubx(self):
        hdr, clsid, msgid, lenb, plb, cks = self.read_UBX()
        if plb is None or clsid is None:
            return
        raw_message = hdr + clsid + msgid + lenb + plb + cks
        self.__save_raw__(raw_message)
        if not check_cks(raw_message):
            return
        msg_class = UBXUnpacker.UbxMessage.byte_find(clsid, msgid)
        if msg_class != UBXUnpacker.UbxMessage:
            parsed = msg_class(plb, datetime.now(tz=Constants.tz_moscow))
        else:
            parsed = UBXReader.parse(raw_message)
            # parsed = msg_class(datetime.now(tz=Constants.tz_moscow))
        self.__save_parsed__(parsed)
        return parsed

    def parse_nmea(self):
        raw_message = self.stream.readline()
        self.__save_raw__(raw_message)
        nmea_type = NMEAUnpacker.NmeaMessage.get_nmea_type(raw_message.decode('utf-8'))
        msg_class = NMEAUnpacker.NmeaMessage.find(nmea_type)
        if msg_class != NMEAUnpacker.NmeaMessage:
            parsed = msg_class(raw_message.decode('utf-8'))
        else:
            parsed = NMEAReader.parse(b'$' + raw_message)
        self.__save_parsed__(parsed)
        return parsed

    def __save_raw__(self, raw_message):
        if self.print_raw:
            print(f'{datetime.now(tz=Constants.tz_moscow)}: {raw_message}')
        with open(self.raw_logger, 'a') as file:
            file.write(str(raw_message) + '\n')

    def __save_parsed__(self, parsed_message):
        if self.print_parsed:
            print(f'{datetime.now(tz=Constants.tz_moscow)}: {parsed_message}')
        with open(self.parsed_logger, 'a') as file:
            file.write(str(parsed_message) + '\n')


class GPSDataPrinter:
    @staticmethod
    def print(type, data=None):
        if data is None:
            print(type)
        else:
            print(type, data)


if __name__ == "__main__":
    Reader = GPSReader()
    Storage = GPSStorage()

    start_year = datetime(2023, 1, 1)
    with open(GPSReader.parsed_logger, 'a') as logger:
        logger.write('------------------------------Start------------------------------\n')

    for counter, parsed_data in enumerate(Reader):
        Storage.update(parsed_data)

    pass
