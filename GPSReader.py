import math
from math import sin, cos, tan
from datetime import datetime, timedelta
from serial import Serial

from GPSStorage import GPSStorage
from UBXUtils import *

import numpy as np, pandas as pd

# TODO: убрать современем
from pynmeagps import NMEAReader
from pyubx2 import UBXReader


class GPSReader:
    print_all_nmea = True
    print_all_ubx = True
    print_raw_nmea = False
    print_raw_ubx = False
    print_noises = False

    nmea_raw_file = 'nmea_raw.txt'
    nmea_parsed_file = 'nmea_parsed.txt'
    ubx_raw_file = 'ubx_raw.txt'
    ubx_parsed_file = 'ubx_parsed.txt'


    raw_messages = []
    parsed_messages = []

    PoolQ = [
        # POOLMessages.RST,
        POOLMessages.GLO,
        POOLMessages.ALM,
        POOLMessages.EPH,
        # POOLMessages.GLO,
        # POOLMessages.ON_ALL,
        # POOLMessages.RST,
        # b'\x06\x04\x04\x00\xFF\xFF\x00\x00' # CFG-RST
    ]
    Pool_step = 60

    def __init__(self, port="/dev/ttyS0", baudrate=9600, timeout=1):  # timeout влияет на буфер, 0.1 мало
        self.stream = Serial(port, baudrate, timeout=timeout)
        self.tune()
        self.counter = 0

    def read_next_message(self):
        self.counter += 1
        if self.counter % self.Pool_step == 1 and self.PoolQ:
            self.pool_next()
        if self.counter % 5000 == 0:
            self.PoolQ += [POOLMessages.ALM, POOLMessages.EPH]

        hdr1 = self.stream.read(1)
        if hdr1 == b'\xb5':
            return self.parse_ubx()
        elif hdr1 == b'$':
            self.parse_nmea()
        else:
            if self.print_noises:
                print(hdr1)

    def tune(self):
        for message in MSG2set:
            print(f'\tTune: {message}')
            self.send(message)

    def send(self, message):
        self.stream.write(message)

    def pool_next(self):
        cmd = self.PoolQ.pop()
        print(f'\tPool: {cmd}')
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

    def parse_ubx(self, print_all=False, print_raw=False):
        hdr, clsid, msgid, lenb, plb, cks = self.read_UBX()
        current_time = datetime.now()
        raw_message = hdr + clsid + msgid + lenb + plb + cks
        if plb is None:
            return
        self.__save_raw__(raw_message, 'u')

        if self.print_raw_ubx:
            print(f'{current_time}: {raw_message}')
        if (clsid, msgid) in functions.keys():
            parsed_message = functions[(clsid, msgid)](plb, current_time)
            self.__save_parsed__(parsed_message, 'u')
            return parsed_message
        else:
            parsed = UBXReader.parse(raw_message)
            if self.print_all_ubx:
                print(parsed) #f'{current_time}: {parsed}')

    def parse_nmea(self, print_all=False, print_raw=False):
        raw_message = b'$' + self.stream.readline()
        parsed = NMEAReader.parse(raw_message)
        self.__save_parsed__(parsed, 'n')
        self.__save_raw__(raw_message, 'n')
        if self.print_all_nmea:
            print(f'{datetime.now()}: {raw_message}')
        if self.print_raw_nmea:
            print(f'{datetime.now()}: {parsed}')

    def __save_raw__(self, raw_message, type):
        self.raw_messages.append(raw_message)
        if type == 'u':
            file = self.ubx_raw_file
        else:
            file = self.nmea_raw_file
        with open(file, 'a') as file:
            file.write(str(raw_message) + '\n')

    def __save_parsed__(self, parsed_message, type):
        self.parsed_messages.append(parsed_message)
        if type == 'u':
            file = self.ubx_parsed_file
        else:
            file = self.nmea_parsed_file
        with open(file, 'a') as file:
            file.write(str(parsed_message) + '\n')


class GPSDataPrinter:
    @staticmethod
    def print(type, data=None):
        if data==None:
            print(type)
        else:
            print(type, data)



if __name__ == "__main__":
    Reader = GPSReader()
    Storage = GPSStorage()

    start_year = datetime(2023, 1, 1)
    time13 = start_year - timedelta(days=1) + timedelta(days=345.09278128)

    counter = 0

    while True:
        counter += 1

        t = 503278
        # print(Storage)
        # if counter == -1:
        #     eph_pd = pd.DataFrame([line for line in Storage.EPH[1:] if line], columns=Storage.EPH[0])
        #     alm_pd = pd.DataFrame([line for line in Storage.ALM[1:] if line], columns=Storage.ALM[0])
        #     print(eph_pd)
        #     print(alm_pd)
        #     with open('eph1.csv', 'w') as feph:
        #         feph.write(eph_pd.to_csv())
        #     with open('alm1.csv', 'w') as feph:
        #         feph.write(alm_pd.to_csv())
        #     # eph_pd.to_csv('eph.csv')
        #     # alm_pd.to_csv('alm.csv')
        #     print(Storage)
            # print('\n EPH coord\n')
            # for eph in Storage.EPH[1:]:
            #     if eph:
            #         sat = calc_sat_eph(eph, t)
            #         print(f'{eph[0]}, {np.array(sat[:4]) * 180/pi % 360 }', {sat[4:]}, )
            #
            # print('\n ALM coord\n')
            # for alm in Storage.ALM[1:]:
            #     if alm:
            #         sat = calc_sat_alm(alm, t)
            #         print(f'{alm[0]}, {np.array(sat[:4]) * 180 / pi % 360}', {sat[4:]}, )

            # a=0



        parsed = Reader.read_next_message()
        if not parsed:
            continue
        print(parsed)
        # if not isinstance(parsed, list) or not isinstance(parsed, tuple):
        #     GPSDataPrinter.print(parsed)
        # else:
        #     type, data = parsed
        #     Storage.update(type, data)
        #     GPSDataPrinter.print(type, data)

    pass
