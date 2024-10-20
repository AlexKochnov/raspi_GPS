import struct
from datetime import datetime
from time import sleep
import traceback
from serial import Serial

from Messages import BaseMessages
from Messages import UBXMessages
from Messages.NMEAMessages import tune_baudRate_message, NmeaMessage
from Utils import Settings, Save
from Utils.TimeStamp import TimeStamp
from Utils.Settings import START_ID
from Utils.GNSS import GNSS

# TODO: delete in long future
from pyubx2 import UBXReader

class Reader:
    """
    Класс для организации чтения данных с порта, его настройки и распаковки полученных сообщений
    read_counter: int - счетчик прочитанных сообщений
    pool_counter: int - счетчик отправленных сообщений
    stream - поток чтения данных
    file: bool - флаг чтения из файла, а не из потока
    """
    read_counter: int = 0
    pool_counter: int = 0

    stream = None
    file = False

    def __init__(self, port=Settings.SerialPort, baudRate=Settings.BaudRate, timeout=Settings.timeout, file=None):
        """
        Инициализация объекта управления портом и чтением сообщений
        :param port: str - порт для чтения
        :param baudRate: int - частота чтения
        :param timeout: int - задержка timeout
        :param file: str - путь к файлу логов для чтения при необходимости
        """
        self.port = port
        self.baudRate = baudRate
        self.timeout = timeout
        if file is not None:
            self.stream = open(file, 'rb')
            self.file = True
        else:
            self.tune_module(baudRate)

    def __iter__(self) -> UBXMessages or UBXReader or NmeaMessage or str:
        """
        Запуск итератора для получения сообщений ц цикле
        :return: UBXMessages or UBXReader or NmeaMessage or str - распакованное сообщение
        """
        while True:
            yield self.next()

    def send(self, message: bytes):
        """
        Функция отправки команды
        :param message: bytes - команда
        :return:
        """
        if not self.file:
            self.stream.write(message)

    def next(self) -> UBXMessages or UBXReader or NmeaMessage or str:
        """
        Функция для действий нового такта - отправка команды при необходимости и чление следующего сообщения
        :return: UBXMessages or UBXReader or NmeaMessage or str - распакованное сообщение
        """
        if self.read_counter % Settings.ReaderPoolStep == Settings.ReaderPoolStart:
            self.pool_next()
        return self.read_next_message()

    def new_stream(self, baudrate: int=None):
        """
        Открытие порта заново
        :param baudrate: int - частота порта
        :return:
        """
        if not baudrate:
            baudrate = self.baudRate
        if self.stream:
            self.stream.close()
        sleep(0.3)
        self.stream = Serial(port=self.port, baudrate=baudrate, timeout=self.timeout)

    @staticmethod
    def parse_full_line(line: bytes) -> UBXMessages or str:
        """
        Функция распаковки сообщения не из порта
        :param line: bytes - распаковываемое сообщение
        :return: UBXMessages or str - распакованное сообщение или пустая строка
        """
        hdr, clsid, msgid, lenb, plb = line[:2], line[2:3], line[3:4], line[4:6], line[6:]
        msg_class = UBXMessages.UbxMessage.byte_find(clsid, msgid)
        if msg_class != UBXMessages.UbxMessage:
            parsed = msg_class(plb, TimeStamp())
        else:
            parsed = ""
        return parsed

    def tune_module(self, baudRate: int):
        """
        Функция для настройки модуля
        :param baudRate: int - частота связи по serial порты
        :return:
        """
        self.new_stream(baudRate)
        self.stream.write(tune_baudRate_message(baudRate=Settings.BaseBaudRate))
        self.new_stream(Settings.BaseBaudRate)
        for message in BaseMessages.tune_messages:
            print(f'\tTune: {message}')
            self.stream.write(message)
        sleep(0.5)
        self.stream.write(tune_baudRate_message(baudRate=Settings.BaudRate))
        sleep(0.2)
        self.new_stream(baudRate)

    def pool_next(self):
        """
        Функция отправки команды на приемник
        :return:
        """
        if not BaseMessages.pool_messages:
            return
        cmd = BaseMessages.pool_messages[self.pool_counter % len(BaseMessages.pool_messages)]
        self.pool_counter += 1
        self.send(b'\xb5b' + cmd + UBXMessages.calc_ubx_checksum(cmd))
        print(f'\t#Pool: {cmd}')

    def read_next_message(self) -> UBXMessages or UBXReader or NmeaMessage or str:
        """
        Функция чтения нового сообщения:
        :return: UBXMessages or UBXReader or NmeaMessage or str - распакованное сообщение
        """
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

    def parse_ubx(self) -> UBXMessages or UBXReader:
        """
        Чтение и распаковка сообщения типа UBX
        :return: распакованное сообщение
        """
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
            parsed = msg_class(plb, TimeStamp())
            parsed.raw = hdr + clsid + msgid + lenb + plb + cks
        else:
            parsed = UBXReader.parse(raw_message)
        Save.save_parsed(parsed)
        if Settings.PrintParsedFlag:
            print(parsed)
        return parsed

    def read_ubx(self ) -> tuple[bytes, bytes, bytes, bytes, bytes, bytes]:
        """
        Чтение сообщения типа UBX
        :return: tuple[bytes * 6]
            - hdr: bytes - заголовок
            - clsid: bytes - id класса сообщения
            - msgid: bytes - id сообщения внутри класса
            - lenb: bytes - длинна plb части сообщения в виде байт
            - plb: bytes - часть с данными
            - cks: bytes - чек-сумма
        """
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

    def parse_nmea(self) -> NmeaMessage or str:
        """
        Чтение и распаковка сообщения типа NMEA
        :return: распакованное сообщение
        """
        raw_message = b'$' + self.stream.readline()
        Save.save_raw(raw_message)
        if Settings.PrintRawFlag:
            print(raw_message)
        msg = raw_message.decode()
        msg_class = NmeaMessage.find(NmeaMessage.get_head(msg))
        if msg_class != NmeaMessage:
            parsed = msg_class(msg)
        else:
            parsed = msg
        Save.save_parsed(str(parsed).replace('\n', ''))
        if Settings.PrintParsedFlag:
            print(str(parsed).replace('\n', ''))
        return parsed
