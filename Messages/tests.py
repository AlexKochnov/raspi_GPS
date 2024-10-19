from unittest.mock import patch, MagicMock
import pytest

from Messages.test_results import results
from Messages.Reader import Reader
from Utils import Settings

def mock_serial_port(func):
    def wrapper(*args, **kwargs):
        with patch('serial.Serial') as mock_serial:
            mock_serial_instance = MagicMock()
            mock_serial_instance.is_open = True
            mock_serial.return_value = mock_serial_instance
            return func(mock_serial_instance, *args, **kwargs)
    return wrapper

def dict_compare(real, test):
    for key, value in real.items():
        assert value == test[key]

def test_reader_ubx():
    """
    Тест всех сообщений UBX при чтении из файлов логов
    :return:
    """
    reader = Reader(file='test_data.log')
    Settings.PrintParsedFlag = False
    Settings.PrintRawFlag = False
    Settings.SaveRawFlag = False
    Settings.SaveParsedFlag = False

    for line in results:
        res = reader.next()
        dict_compare(line[0], res.data)

        assert type(line[1]) == type(res.satellites)
        if type(line[1]) is list:
            for sat_res in line[1]:
                for sat_test in res.satellites:
                    if sat_test['svId'] == sat_res['svId']:
                        dict_compare(sat_res, sat_test)

        if len(line) == 3:
            dict_compare(line[2], res.signal)


if __name__ == "__main__":
    pytest.main()

