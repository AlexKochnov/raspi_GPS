from Utils.Settings import *


def save_raw(message):
    __save__(message, raw_logger, SaveRawFlag, 'ab')


def save_parsed(message):
    __save__(message, parsed_logger, SaveParsedFlag, 'a')


def __save__(message, file, flag, mode='a'):
    if flag:
        with open(file, mode) as logger:
            if 'b' not in mode:
                print(message, file=logger)
            else:
                logger.write(message)
