from datetime import datetime

from Messages.Reader import Reader
from Utils import Settings
from Storage.DataStore import DataStore
from Utils.GNSS import GNSS
from Utils.Settings import START_ID


def start_DataStore():
    # reader = Reader("COM5")
    # reader = Reader(file='../rawOLD.log')
    t1 = datetime.now()
    reader = Reader(file='raw_kuzm2.log')
    # reader = Reader(file='raw1.log')
    # reader = Reader(file='ira_messages3.txt')
    # storage = DataStore(GNSS.GPS)
    storage = DataStore(GNSS.GPS, GNSS.GLONASS, multi_gnss_task=True)
    counter = 1
    STEP = 3000
    # STEP = 100

    # import pymap3d as pm
    # # Settings.LLA = [55.569861111111116, 38.805027777777774, 140] # Дача
    # Constants.LLA = [55.929684333333334, 37.7886145,160 ]
    # Constants.ECEF = pm.geodetic2ecef(*Constants.LLA)
    # FLAG = False

    Settings.PrintNoiseFlag = False
    Settings.PrintParsedFlag = False
    # Settings.PrintParsedFlag = True
    # Settings.SaveRawFlag = False
    # Settings.SaveParsedFlag = False
    empty_parsed = 0
    for parsed in reader:
        counter += 1
        storage.update(parsed)
        a=0
        if not parsed:
            empty_parsed += 1
            if empty_parsed > 10:
                storage.serialize()
                print(counter)
                print(START_ID)
                exit()
        if counter % STEP == 0:

            print(counter)
            b=0
            # storage.serialize()
            # print(Settings.START_ID)

            # exit()
    t2 = datetime.now()
    storage.serialize()
    print(f'start_id: {Settings.START_ID}')
    print(f'{counter} messages')
    print((t2 - t1).total_seconds(), 'seconds for processing')
    print((datetime.now()-t2).total_seconds(), 'seconds for saving')
    a=0

if __name__ == '__main__':
    start_DataStore()

    pass