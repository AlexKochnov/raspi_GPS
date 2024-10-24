import numpy as np
import pandas as pd
from numpy.linalg import norm

from Storage.Filters import FederatedKalmanFilter, LocalKalmanFilter, Entry


def serialize_entry_to_pd_df(entry: Entry) -> dict:
    """
    Функция сериализации записи об измерении или расчете местоположения
    :param entry: Entry - преобразуемая запись
    :return: dict - преобразованный словарь выбранных атрибутов
    """
    return {
        'TOW': entry.stamp.TOW,
        'week': entry.stamp.week,
        'state': entry.state,
        'P': entry.P,
        'normP': norm(entry.P),
        'GDOP': entry.GDOP,
        'fval': entry.fval,
        'scores': entry.scores,
        'derivative': entry.derivative,
        'norm_derivative': norm(entry.derivative),
        'sat_count': entry.sat_count,
        'source_gnss': entry.source.gnssId.name,
        'source_dataType': entry.source.dataType.name,
    }

def my_float(a, n=6):
    if np.isnan(a):
        return 'nan'
    elif np.isinf(a):
        if a > 0:
            return 'inf'
        else:
            return '-inf'
    else:
        return f'{a:.{n}f}' if n >= 0 else f'{a:.{-n}e}'

def serialize_entry_to_table(entry: Entry, format: str) -> dict:
    """
    Функция сериализации записи об измерении или расчете местоположения
    :param entry: Entry - преобразуемая запись
    :return: dict - преобразованный словарь выбранных атрибутов
    """
    res = {
        'TOW': round(entry.stamp.TOW),
        'week': entry.stamp.week,
        'normP': my_float(norm(entry.P)),
        'GDOP': my_float(entry.GDOP, 2),
        'fval': my_float(entry.fval),
        'scores': my_float(LocalKalmanFilter.calc_last_score(entry.scores), 2),
        'norm_derivative': my_float(norm(entry.derivative)),
        'sat_count': my_float(entry.sat_count, 0),
        'gnss': entry.source.gnssId.name,
        'dataType': entry.source.dataType.name,
    }
    if format == 'XYZ':
        res |= {
            'X': my_float(entry.state[0]),
            'Y': my_float(entry.state[1]),
            'Z': my_float(entry.state[2]),
        }
    elif format == 'LLA':
        res |= {
            'lat': my_float(entry.state[0], 5),
            'lon': my_float(entry.state[1], 5),
            'alt': my_float(entry.state[2]),
        }
    return res

def serialize_any_KF(kf: FederatedKalmanFilter or LocalKalmanFilter) -> pd.DataFrame:
    """
    Функция сериализации объекта Федеративного или Локального фильтров Калмана
    :param kf: FederatedKalmanFilter or LocalKalmanFilter - сериализуемый фильтр
    :return: pd.DataFrame - таблица с получаемыми данными
    """
    return pd.DataFrame([serialize_entry_to_pd_df(entry) for entry in kf.history])

def serialize_FKF(fkf: FederatedKalmanFilter):
    """
    Функция сериализации объекта федеративного фильтра Калмана
    :param fkf: FederatedKalmanFilter - сериализуемый фильтр
    :return: pd.DataFrame - таблица с получаемыми данными
    """
    return serialize_any_KF(fkf)

def serialize_LKF(lkf: LocalKalmanFilter):
    """
    Функция сериализации объекта федеративного фильтра Калмана
    :param lkf: LocalKalmanFilter - сериализуемый фильтр
    :return: pd.DataFrame - таблица с получаемыми данными
    """
    return serialize_any_KF(lkf)
