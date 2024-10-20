from time import sleep

from Utils import Constants
from UBXUtils import reset_module, tune_baudRate

if __name__ == '__main__':
    port = Constants.SerialPort
    baudrate = Constants.BaudRate
    timeout = 1

    reset_module(port, baudrate, timeout)
    sleep(0.01)
    tune_baudRate(port, baudrate, timeout)

    print('Module is reset')

