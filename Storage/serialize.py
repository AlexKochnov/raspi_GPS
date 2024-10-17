import numpy as np
import pandas as pd
from numpy.linalg import norm

from Storage.Filters import FederatedKalmanFilter, LocalKalmanFilter, Entry


def serialize_entry(entry: Entry) -> dict:
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

def serialize_any_KF(kf: FederatedKalmanFilter or LocalKalmanFilter) -> pd.DataFrame:
    """
    Функция сериализации объекта Федеративного или Локального фильтров Калмана
    :param kf: FederatedKalmanFilter or LocalKalmanFilter - сериализуемый фильтр
    :return: pd.DataFrame - таблица с получаемыми данными
    """
    return pd.DataFrame([serialize_entry(entry) for entry in kf.history])

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
