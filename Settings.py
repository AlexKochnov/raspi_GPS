BaudRate = 115200
BaseBaudRate = 9600
SerialPort = "/dev/ttyS0"
timeout = 1

MinimizingSatellitesCount = 8
MinimumMinimizingSatellitesCount = 4

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
max_calc_time = 0.150

GUI_ON = False


