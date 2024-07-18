import UBXMessages
from Reader import Reader
from Storage import Storage
from PyQt5.QtCore import QThread, pyqtSignal


class AppReaderThread(QThread):
    signal = pyqtSignal(bool, object)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.reader = Reader(self.port)
        self.storage = Storage()

    def run(self):
        for parsed in self.reader:
            flag = self.storage.update(parsed)
            self.signal.emit(flag, parsed)


port = "COM3"

reader = AppReaderThread(port)

reader.signal.connect
