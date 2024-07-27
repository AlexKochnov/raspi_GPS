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

optimize_methods = {
    'LM': 'Levenberg-Marquardt',
    'SQP': 'SQP',
    'TRF': 'trf',
    'TC': 'trust-constr',
    'DB': 'DogBox',
}
