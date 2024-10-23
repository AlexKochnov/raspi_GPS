from datetime import datetime

from pynmeagps import NMEAMessage

from Messages.Reader import Reader
from Utils import Settings
from Storage.DataStore import DataStore
from Utils.GNSS import GNSS


def start_DataStore():
    t1 = datetime.now()
    # reader = Reader("COM5")
    reader = Reader(file='raw_kuzm2.log')
    storage = DataStore(GNSS.GPS, GNSS.GLONASS, multi_gnss_task=True)

    counter = 1
    STEP = 3000

    Settings.PrintNoiseFlag = False
    Settings.PrintParsedFlag = False
    # Settings.SaveRawFlag = False
    # Settings.SaveParsedFlag = False

    msgs = []

    empty_parsed = 0
    for parsed in reader:
        msgs.append(parsed)
        counter += 1
        storage.update(parsed)
        if not parsed:
            empty_parsed += 1
            if empty_parsed > 10:
                break
        if counter % STEP == 0:
            print(counter)
    t2 = datetime.now()
    storage.serialize()
    print(f'start_id: {Settings.START_ID}')
    print(f'{counter} messages')
    print((t2 - t1).total_seconds(), 'seconds for processing')
    print((datetime.now()-t2).total_seconds(), 'seconds for saving')

if __name__ == '__main__':
    start_DataStore()
