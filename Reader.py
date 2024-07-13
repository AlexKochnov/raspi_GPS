import struct
from time import sleep
from serial import Serial

import Constants
import Save
import Messages
import Settings
import UBXMessages

from NMEAMessages import tune_baudRate_message
from Storage import Storage

# TODO: delete
from pyubx2 import UBXReader


class Reader:
    storage: Storage = Storage()
    read_counter = 0
    pool_counter = 0

    TOW = None

    def __init__(self, port=Settings.SerialPort, baudRate=Settings.BaudRate, timeout=1):
        self.stream = Serial(port, baudRate, timeout=timeout)
        self.tune_module(baudRate)
        a=0
        # baudRate = 9600
        # self.tune_module(port, baudRate, timeout)
        # self.stream = Serial(port, baudRate, timeout=timeout)

    def __iter__(self):
        while True:
            yield self.next()

    def update_TOW(self, TOW):
        self.TOW = TOW

    def send(self, message):
        self.stream.write(message)

    def next(self):
        if self.read_counter % Settings.ReaderPoolStep == Settings.ReaderPoolStart:
            self.pool_next()
        return self.read_next_message()

    @staticmethod
    def tune_baudRate_module(FromBaudRate, ToBaudRate, port=Settings.SerialPort, timeout=1):
        cmd = tune_baudRate_message(baudRate=ToBaudRate)
        with Serial(port, FromBaudRate, timeout=timeout) as stream:
            stream.write(cmd)
        sleep(0.1)

    def tune_module2(self, port, baudRate, timeout):
        cmd = tune_baudRate_message(baudRate=Settings.BaseBaudRate)
        with Serial(port, baudRate, timeout=timeout) as stream:
            stream.write(cmd)
        sleep(0.1)
        with Serial(port, Settings.BaseBaudRate, timeout=timeout) as stream:
            for message in Messages.tune_messages:
                print(f'\tTune: {message}')
                stream.write(message)
        self.tune_baudRate_module(Settings.BaseBaudRate, baudRate, port, timeout)

    def tune_module(self, baudRate):
        self.stream.baudrate = baudRate
        print('115200, s67', self.stream.read(50))
        self.stream.write(tune_baudRate_message(baudRate=Settings.BaseBaudRate))
        sleep(1)
        self.stream.baudrate = Settings.BaseBaudRate
        print('9600, s71', self.stream.read(50))
        for message in Messages.tune_messages:
            print(f'\tTune: {message}')
            self.stream.write(message)
        sleep(1)
        self.stream.baudrate = baudRate
        print('115200, s76', self.stream.read(50))


    def tune(self):
        for message in Messages.tune_messages:
            print(f'\tTune: {message}')
            self.stream.send(message)

    def pool_next(self):
        if not Messages.pool_messages:
            return
        cmd = Messages.pool_messages[self.pool_counter % len(Messages.pool_messages)]
        self.send(cmd)
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
            parsed = msg_class(plb)#, self.TOW or -1)
        else:
            parsed = UBXReader.parse(raw_message)
        Save.save_raw(parsed)
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
        # msg_class = NMEAUnpacker.NmeaMessage.find(nmea_type)
        # if msg_class != NMEAUnpacker.NmeaMessage:
        #     parsed = msg_class(raw_message.decode('utf-8'))
        # else:
        # parsed = NMEAReader.parse(b'$' + raw_message)
        # self.__save_parsed__(parsed)
        # TODO: добавить что-то, когда будет нужна распаковка NMEA
        parsed = raw_message
        Save.save_raw(parsed)
        if Settings.PrintParsedFlag:
            print(parsed)
        return parsed


if __name__ == '__main__':
    reader = Reader()
    storage = Storage()

    for parsed in reader:
        storage.update(parsed)
