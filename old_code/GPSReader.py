from datetime import datetime
from time import sleep

from pyubx2 import UBXReader
# from matplotlib import pyplot as plt

import argparse

import NMEAUnpacker
import UBXUnpacker
from GPSStorage import GPSStorage
from UBXUtils import *

from Utils import Constants

# import matplotlib.pyplot as plt

# TODO: убрать современем
from pynmeagps import NMEAReader


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

        # POOLMessages.CFG_ESRC,
        # POOLMessages.CFG_GNSS,
        # POOLMessages.TIM_SMEAS,

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

    @staticmethod
    def reset_module(port=Constants.SerialPort, baudrate=Constants.BaudRate, timeout=1):
        reset_module(port, baudrate, timeout)
        sleep(0.1)
        tune_baudRate(port, baudrate, timeout)
        sleep(0.1)

    @staticmethod
    def tune_baudRate_module(port=Constants.SerialPort, baudrate=Constants.BaudRate, timeout=1):
        tune_baudRate(port, baudrate, timeout)
        sleep(0.1)

    def __init__(self, port=Constants.SerialPort, baudrate=9600, timeout=1):#, reset: bool=False):  # timeout влияет на буфер, 0.1 мало
        # reset = True
        # if reset:
        #     reset_module(port, baudrate, timeout)
        #     sleep(0.1)
        #     tune_baudRate(port, baudrate, timeout)
        #     sleep(0.1)
        self.tune_baudRate_module(port, baudrate, timeout)
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--reset', action='store_true',
                        help='Reset the module with CFG-RST and config baudRate to 115200')
    args = parser.parse_args()
    if args.reset:
        GPSReader.reset_module()

    # start_year = datetime(2023, 1, 1)
    with open(GPSReader.parsed_logger, 'a') as logger:
        logger.write('------------------------------Start------------------------------\n')

    Reader = GPSReader()
    Storage = GPSStorage()

    for counter, parsed_data in enumerate(Reader):
        Storage.update(parsed_data)

    pass






