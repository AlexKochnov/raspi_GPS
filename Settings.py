from random import randint

BaudRate = 115200
BaseBaudRate = 9600
SerialPort = "/dev/ttyS0"
timeout = 1

MaximumMinimizingSatellitesCount = 8
MinimumMinimizingSatellitesCount = 4

MinimumNavigationSatelliteScore = 15

ReaderPoolStep = 200
ReaderPoolStart = 15

SaveRawFlag = True
SaveParsedFlag = True
raw_logger = 'raw.log'
parsed_logger = 'parsed.log'

PrintNoiseFlag = True
PrintParsedFlag = True
PrintRawFlag = False

used_method = 'LM'
using_methods = [
    'LM',
    'SQP',
    'TRF',
    'TC',
    'DB',
]
max_calc_time = 0.4

GUI_ON = False

LastDtDelay = 0

START_ID = randint(0, 100000)


