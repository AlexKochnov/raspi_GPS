import struct
from time import sleep
import traceback

from pynmeagps import NMEAMessage
from serial import Serial

import Constants
import Save
import Messages
import Settings
import UBXMessages

from NMEAMessages import tune_baudRate_message, NmeaMessage
from Storage import Storage

# TODO: delete
from pyubx2 import UBXReader

from TimeStamp import TimeStamp


class Reader:
    # storage: Storage = Storage()
    read_counter = 0
    pool_counter = 0

    TOW = None
    stream = None
    file= False

    def __init__(self, port=Settings.SerialPort, baudRate=Settings.BaudRate, timeout=Settings.timeout, file=None):
        self.port = port
        self.baudRate = baudRate
        self.timeout = timeout
        if file is not None:
            self.stream = open(file, 'rb')
            self.file = True
        else:
            self.tune_module(baudRate)
        a=0

    def __iter__(self):
        while True:
            yield self.next()

    def update_TOW(self, TOW):
        self.TOW = TOW

    def send(self, message):
        if not self.file:
            self.stream.write(message)

    def next(self):
        if self.read_counter % Settings.ReaderPoolStep == Settings.ReaderPoolStart:
            self.pool_next()
        return self.read_next_message()

    def new_stream(self, baudrate=None):
        if not baudrate:
            baudrate = self.baudRate
        if self.stream:
            self.stream.close()
        sleep(0.3)
        self.stream = Serial(port=self.port, baudrate=baudrate, timeout=self.timeout)

    def tune_module(self, baudRate):
        self.new_stream(baudRate)
        self.stream.write(tune_baudRate_message(baudRate=Settings.BaseBaudRate))
        sleep(0.2)
        # self.stream.read(100)
        # print(f'Baudrate switched to: {Settings.BaseBaudRate}, data: {self.stream.read(3000)}')
        self.new_stream(Settings.BaseBaudRate)
        # self.stream.read(100)
        # print(f'Baudrate: {Settings.BaseBaudRate}, data: {self.stream.read(3000)}')
        for message in Messages.tune_messages:
            print(f'\tTune: {message}')
            self.stream.write(message)
        sleep(0.5)
        self.stream.write(tune_baudRate_message(baudRate=Settings.BaudRate))
        sleep(0.2)
        # self.stream.read(100)
        # print(f': {Settings.BaudRate}, data: {self.stream.read(3000)}')
        self.new_stream(baudRate)
        # self.stream.read(100)
        # print(f'Baudrate: {Settings.BaudRate}, data: {self.stream.read(3000)}')
        # for message in Messages.tune_messages:
        #     print(f'\tTune: {message}')
        #     self.stream.write(message)


    def pool_next(self):
        if not Messages.pool_messages:
            return
        cmd = Messages.pool_messages[self.pool_counter % len(Messages.pool_messages)]
        self.pool_counter += 1
        self.send(b'\xb5b' + cmd + UBXMessages.calc_ubx_checksum(cmd))
        print(f'\t#Pool: {cmd}')

    def read_next_message(self):
        try:
            hdr1 = self.stream.read(1)
            if hdr1 == b'\xb5':
                self.read_counter += 1
                return self.parse_ubx()
            elif hdr1 == b'$':
                self.read_counter += 1
                return self.parse_nmea()
            else:
                if Settings.PrintNoiseFlag:
                    print(hdr1)
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            a=0

    def parse_ubx(self):
        hdr, clsid, msgid, lenb, plb, cks = self.read_ubx()
        if plb is None or clsid is None:
            return
        raw_message = hdr + clsid + msgid + lenb + plb + cks
        if Settings.PrintRawFlag:
            print(raw_message)
        Save.save_raw(raw_message)
        if not UBXMessages.check_ubx_checksum(raw_message):
            return
        msg_class = UBXMessages.UbxMessage.byte_find(clsid, msgid)
        if msg_class != UBXMessages.UbxMessage:
            parsed = msg_class(plb, TimeStamp())#, self.TOW or -1)
        else:
            parsed = UBXReader.parse(raw_message)
        Save.save_parsed(parsed)
        if Settings.PrintParsedFlag:
            print(parsed)
        return parsed

    def read_ubx(self):
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

    def parse_nmea(self):
        raw_message = b'$' + self.stream.readline()
        Save.save_raw(raw_message)
        if Settings.PrintRawFlag:
            print(raw_message)
        # nmea_type = NMEAUnpacker.NmeaMessage.get_nmea_type(raw_message.decode('utf-8'))
        msg = raw_message.decode()
        msg_class = NmeaMessage.find(NmeaMessage.get_head(msg))
        if msg_class != NmeaMessage:
            # try:
            parsed = msg_class(msg)
            # except:
            #     parsed = ''
        else:
            parsed = msg
        # parsed = NMEAReader.parse(b'$' + raw_message)
        # self.__save_parsed__(parsed)
        # TODO: добавить что-то, когда будет нужна распаковка NMEA
        # parsed = raw_message
        # try:
        Save.save_parsed(str(parsed).replace('\n', ''))
        # except:
        #     pass
        if Settings.PrintParsedFlag:
            print(str(parsed).replace('\n', ''))
        return parsed


if __name__ == '__main__':

    # reader = Reader("COM3")
    reader = Reader(file='../rawOLD.log')
    # reader = Reader(file='ira_messages3.txt')
    storage = Storage()
    counter = 1
    STEP = 4000
    # STEP = 100

    import pymap3d as pm
    # Settings.LLA = [55.569861111111116, 38.805027777777774, 140] # Дача
    Constants.LLA = [55.929684333333334, 37.7886145,160 ]
    Constants.ECEF = pm.geodetic2ecef(*Constants.LLA)
    FLAG = False

    Settings.PrintNoiseFlag = False
    Settings.PrintParsedFlag = False
    Settings.SaveRawFlag = False
    Settings.SaveParsedFlag = False

    # while True:
    #     reader.stream.readline()
    #     counter += 1
    #     if counter > 1.7e6:
    #         break
    # print(counter)

    for parsed in reader:
        counter += 1

        if counter % STEP == 0:
            print(f'STEP: {counter}, time: {storage.time_stamp}')
            with open('LOGGER_TRASH.txt', 'a') as logger_trash:
                print(f'STEP: {counter}, time: {storage.time_stamp}', file=logger_trash)


        # if sum(storage.ephemeris_data.nav_score > 15) > 3:
        #     b=0
        # if sum(storage.almanac_data.nav_score > 15) > 3:
        #     c=0
        # if isinstance(parsed, UBXMessages.NAV_TIMEGPS):
        #     # print(parsed.data['iTOW'])
        #     # if parsed.data['iTOW'] > 162000 * 1000 and parsed.data['week'] == 2324:
        #     if parsed.data['week'] == 2325 and parsed.data['iTOW'] > 190000 * 1000:
        #         FLAG = True
        #         if parsed.data['iTOW'] > 230000 * 1000:
        #             FLAG = False
        # # if FLAG:
        a=0
        storage.update(parsed)
        a=0
        # if storage.time_stamp.TOW > 162000 and storage.time_stamp.week == 2324:
        #     a=0
        #     break
    a=0


