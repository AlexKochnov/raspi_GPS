import math
from math import sin, cos, tan
from datetime import datetime, timedelta

# from matplotlib import pyplot as plt
from serial import Serial

import UBXUnpacker
from GPSStorage import GPSStorage, calc_sat_alm, calc_sat_eph
from UBXUtils import *

import numpy as np, pandas as pd
# import matplotlib.pyplot as plt

# TODO: убрать современем
from pynmeagps import NMEAReader
from pyubx2 import UBXReader


class GPSReader:
    print_noises = True
    print_parsed = True
    print_raw = False

    raw_logger = 'raw.log'
    parsed_logger = 'parsed.log'

    # TODO: modify to command class
    PoolQ = [
        # POOLMessages.RST,
        # POOLMessages.GLO,
        # POOLMessages.RATE_GET,
        # POOLMessages.RATE_SET,
        POOLMessages.EPH,
        # POOLMessages.RAWX,
        POOLMessages.ALM,
        # POOLMessages.GNSS_check,
        # POOLMessages.MON_GNSS,
        # POOLMessages.GLO,
        # POOLMessages.ON_ALL,
        # POOLMessages.RST,
        # b'\x06\x04\x04\x00\xFF\xFF\x00\x00' # CFG-RST
    ]

    Pool_step = 150
    counter = 0
    counetr2 = 0

    def __init__(self, port="/dev/ttyS0", baudrate=9600, timeout=1):  # timeout влияет на буфер, 0.1 мало
        self.stream = Serial(port, baudrate, timeout=timeout)
        self.tune()

    def next(self):
        self.counter += 1
        if self.counter % self.Pool_step == 0:
            self.pool_next()
        else:
            return self.read_next_message()

    def read_next_message(self):
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
        if self.PoolQ:
            # cmd = self.PoolQ.pop()
            cmd = self.PoolQ[self.counetr2 % len(self.PoolQ)]
            self.counetr2 += 1
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

    def parse_ubx(self):
        hdr, clsid, msgid, lenb, plb, cks = self.read_UBX()
        if plb is None or clsid is None:
            return
        raw_message = hdr + clsid + msgid + lenb + plb + cks
        self.__save_raw__(raw_message)
        if not check_cks(raw_message):
            return
        msg_class = UBXUnpacker.Message.byte_find(clsid, msgid)
        if msg_class != UBXUnpacker.Message:
            parsed = msg_class(plb, datetime.now())
        else:
            parsed = UBXReader.parse(raw_message)
            # parsed = msg_class(datetime.now())
        self.__save_parsed__(parsed)
        return parsed

    def parse_nmea(self):
        raw_message = b'$' + self.stream.readline()
        self.__save_raw__(raw_message)
        parsed = NMEAReader.parse(raw_message)
        self.__save_parsed__(parsed)
        return parsed

    def __save_raw__(self, raw_message):
        if self.print_raw:
            print(f'{datetime.now()}: {raw_message}')
        with open(self.raw_logger, 'a') as file:
            file.write(str(raw_message) + '\n')

    def __save_parsed__(self, parsed_message):
        if self.print_parsed:
            print(f'{datetime.now()}: {parsed_message}')
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

    counter = 0

    while True:
        counter += 1

        parsed = Reader.next()
        if not parsed:
            continue
        Storage.update(parsed)
        # for (gnssId, svId), sat in Storage.satellites.items():
        #     if sat.eph and sat.alm and True:
        #         time_stamp = Storage.iTOW / 1000
        #         time = np.arange(time_stamp, time_stamp + 36000, 60)
        #         alm = []
        #         eph = []
        #         for t in time:
        #             alm.append(calc_sat_alm(sat.alm, t, Storage.week))
        #             eph.append(calc_sat_eph(sat.eph, t, Storage.week))
        #         pass
        #
        #         alm = np.array(alm)
        #         eph = np.array(eph)
        #         time = (np.array(time) - time_stamp) / 3600
        #         a = 0
        #         # with open('alm.txt', 'w') as alm_file:
        #         #     alm_file.write(json.dumps(alm))
        #         #
        #         # with open('eph.txt', 'w') as eph_file:
        #         #     eph_file.write(json.dumps(eph))
        #
        ##         fig, axs = plt.subplots(4, 1, figsize=(8, 8))
        #         fig, axs = plt.subplots(3, 1, figsize=(8, 8))
        #
        #
        #         def plot_axs(i, y1, y2, ylabel):
        #             y3 = (y2 - y1)
        #             y2 = y2 * 1e-6
        #             y1 = y1 * 1e-6
        #             axs[i].plot(time, y1, label='alm', linestyle='-', linewidth=2)
        #             axs[i].plot(time, y2, label='eph', linestyle='--', linewidth=2)
        #             axs[i].legend(loc='upper left')
        #             axs[i].set_ylabel(ylabel)
        #
        #             axs2 = axs[i].twinx()
        #             axs2.plot(time, y3, label='e-a', linewidth=1.5, color='gray')
        #             axs2.set_ylabel(ylabel.replace('км', 'м'))
        #             axs2.legend(loc='upper right')
        #
        #
        #         plot_axs(0, alm[:, 0], eph[:, 0], 'X, км')
        #         plot_axs(1, alm[:, 1], eph[:, 1], 'Y, км')
        #         plot_axs(2, alm[:, 2], eph[:, 2], 'Z, км')
        ##         plot_axs(3, alm[:, 3] *1e6, eph[:, 3]*1e6, 'Z, км')
        #         plt.show()
        #         a = 0
                # plot_axs(1, sat.alm_y / 1000, sat.eph_y / 1000, 'Y, км')
                # plot_axs(2, sat.alm_z / 1000, sat.eph_z / 1000, 'Z, км')

                # plt.clf()
                # plt.plot(time, alm[:, 0] / 10**6, label='X, alm', color='b', linestyle='--', linewidth=2)
                # plt.plot(time, alm[:, 1] / 10**6, label='Y, alm', color='g', linestyle='--', linewidth=2)
                # plt.plot(time, alm[:, 2] / 10**6, label='Z, alm', color='r', linestyle='--', linewidth=2)
                # plt.plot(time, eph[:, 0] / 10**6, label='X, eph', color='b', linestyle='-', linewidth=1)
                # plt.plot(time, eph[:, 1] / 10**6, label='Y, eph', color='g', linestyle='-', linewidth=1)
                # plt.plot(time, eph[:, 2] / 10**6, label='Z, eph', color='r', linestyle='-', linewidth=1)
                # plt.xlabel("Время от настоящего, ч")
                # plt.ylabel("Координаты спутника, км")
                # plt.legend(loc='upper left')
                #
                # plt.twinx()
                # plt.plot(time, (alm[:, 0] - eph[:, 0]), label='Xalm - Xeph', color='c', linestyle='-', linewidth=1)
                # plt.plot(time, (alm[:, 1] - eph[:, 1]), label='Yalm - Yeph', color='m', linestyle='-', linewidth=1)
                # plt.plot(time, (alm[:, 2] - eph[:, 2]), label='Zalm - Zeph', color='y', linestyle='-', linewidth=1)
                # plt.ylabel("Разница координат, м")
                # plt.legend(loc='upper right')
                # plt.savefig('position.png', dpi=1000)
                #
                # b = 0
                #
                # plt.clf()
                # plt.plot(time, alm[:, 3], label='Vx, alm', color='b', linestyle='--', linewidth=2)
                # plt.plot(time, alm[:, 4], label='Vy, alm', color='g', linestyle='--', linewidth=2)
                # plt.plot(time, alm[:, 5], label='Vz, alm', color='r', linestyle='--', linewidth=2)
                # plt.plot(time, eph[:, 3], label='Vx, eph', color='b', linestyle='-', linewidth=1)
                # plt.plot(time, eph[:, 4], label='Vy, eph', color='g', linestyle='-', linewidth=1)
                # plt.plot(time, eph[:, 5], label='Vz, eph', color='r', linestyle='-', linewidth=1)
                # plt.xlabel("Время от настоящего, ч")
                # plt.ylabel("Скорости спутника, м/c")
                # plt.legend(loc='upper left')
                #
                # plt.twinx()
                # # plt.plot(time, (alm[:, 3] - eph[:, 3]), label='Vx_alm - Vx_eph', color='c', linestyle='-', linewidth=1)
                # # plt.plot(time, (alm[:, 4] - eph[:, 4]), label='Vy_alm - Vy_eph', color='m', linestyle='-', linewidth=1)
                # plt.plot(time, (alm[:, 5] - eph[:, 5]), label='Vz_alm - Vz_eph', color='y', linestyle='-', linewidth=1)
                # plt.ylabel("Разница скоростей, м/c")
                # plt.legend(loc='upper right')
                # plt.savefig('speed.png', dpi=1000)

                # plt.plot(time, alm[:, 0:3], colors=('b--', 'g--', 'r--'))
                # plt.plot(time, eph[:, 0:3], colors=('b', 'g', 'r'))
                # plt.legend(('Xa', 'Ya', 'Za', 'Xe', 'Ye', 'Ze'))
                # plt.savefig('position.png', dpi=1000)

        # print(parsed)
        # if not isinstance(parsed, list) or not isinstance(parsed, tuple):
        #     GPSDataPrinter.print(parsed)
        # else:
        #     type, data = parsed
        #     Storage.update(type, data)
        #     GPSDataPrinter.print(type, data)

    pass
