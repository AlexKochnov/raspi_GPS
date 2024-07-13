from Settings import *


def save_raw(message):
    __save__(message, raw_logger, SaveRawFlag)


def save_parsed(message):
    __save__(message, parsed_logger, SaveParsedFlag)


def __save__(message, file, flag):
    if flag:
        with open(file, 'a') as logger:
            print(message, file=logger)
