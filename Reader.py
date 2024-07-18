import struct
from time import sleep
from serial import Serial
import traceback

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
    # storage: Storage = Storage()
    read_counter = 0
    pool_counter = 0

    TOW = None
    stream = None

    def __init__(self, port=Settings.SerialPort, baudRate=Settings.BaudRate, timeout=1):
        self.port = port
        self.baudRate = baudRate
        self.timeout = timeout

        self.tune_module(baudRate)

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
        sleep(0.1)
        self.new_stream(Settings.BaseBaudRate)
        for message in Messages.tune_messages:
            print(f'\tTune: {message}')
            self.stream.write(message)
        self.stream.write(tune_baudRate_message(baudRate=Settings.BaudRate))
        sleep(0.1)
        self.new_stream(baudRate)

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

    reader = Reader("COM3")
    storage = Storage()

    for parsed in reader:
        storage.update(parsed)
