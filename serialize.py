import numpy as np
import pandas as pd
from numpy.linalg import norm

from Filters import FederatedKalmanFilter, LocalKalmanFilter, Entry


def serialize_entry(entry: Entry) -> dict:
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
    df = pd.DataFrame(
        data=[serialize_entry(entry) for entry in kf.history]
        # columns=['TOW', 'week', 'state', 'P', 'GDOP', 'fval']
    )
    return df

def serialize_FKF(fkf: FederatedKalmanFilter):
    return serialize_any_KF(fkf)

def serialize_LKF(lkf: LocalKalmanFilter):
    return serialize_any_KF(lkf)
