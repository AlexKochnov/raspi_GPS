import numpy as np
from numpy.linalg import inv, norm

from GNSS import Source
from TimeStamp import TimeStamp


def linear_kalman(xyz_meas, X_k0k0=None, P_k0k0=None):
    """
    Calculate Kalman
    :param xyz_meas: измеренное значение на текущем шаге
    :param X_k0k0: прогноз на предыдущем шаге
    :param P_k0k0: ковариационная матрица на предыдущем шаге
    :return:
    """
    N = len(xyz_meas)

    I = np.eye(N)
    A = np.eye(N)
    B = np.zeros((N, 1))
    C = np.eye(N)
    D = np.zeros((N, N))
    Q = np.eye(N) * 1e-3
    R = np.eye(N) * 1e-3

    U = np.zeros(1)
    # try:
    if X_k0k0 is None or any(np.isnan(X_k0k0)):          # значение на предыдущем шаге
        X_k0k0 = np.zeros(N)
    if P_k0k0 is None or np.isnan(P_k0k0).any():          # ковариационная матрица
        P_k0k0 = np.zeros((N, N))
    if isinstance(P_k0k0, np.matrix):
        P_k0k0 = np.array(P_k0k0)
    if any(np.isnan(xyz_meas)):
        return False, [np.nan, np.nan, np.nan], np.nan # + np.zeros((3,3))
    else:
        Y = np.array(xyz_meas)      # измеренное значение на текущем шаге
    # except Exception as e:
    #     print(e)
    #     a=0

    # прогноз
    X_k1k0 = A @ X_k0k0 + B @ U
    P_k1k0 = A @ P_k0k0 @ A.T + Q

    # коррекция
    Z = Y - C @ X_k1k0
    S = C @ P_k1k0 @ C.T + R
    K = P_k1k0 @ C.T @ inv(S)
    X_k1k1 = X_k1k0 + K @ Z
    P_k1k1 = (I - K @ C) @ P_k1k0 @ (I - K @ C).T + K @ R @ K.T
    # print('MATRIX:', P_k1k1)
    # if np.random.randint(20) == 17:
    #     a=0
    return True, list(X_k1k1), np.matrix(P_k1k1)

class Entry:
    stamp: TimeStamp
    state: np.array
    P: np.array
    GDOP: float
    source: Source
    def __init__(self, stamp: TimeStamp, state: np.array or list, P: np.array, GDOP: float, source: Source):
        self.stamp = stamp
        self.state = np.array(state)
        self.P = np.array(P)
        self.GDOP = GDOP
        self.source = source

class LocalKalmanFilter:
    history: list[Entry]
    def __init__(self):
        self.history = []

    def __bool__(self):
        return bool(self.history)

    def get_last_attr(self, attr_name):
        last = self.get_last()
        if last and hasattr(last, attr_name):
            return None
        return getattr(last, attr_name)

    def get_last(self):
        if len(self.history) == 0:
            return None
        return self.history[-1]

    @property
    def last(self):
        return self.get_last()

    def get_derivative(self):
        if len(self.history) <= 2:
            return np.nan
        return self.history[-1].state - self.history[-2].state

    def update(self, measurements, time_stamp, **kwargs):
        last_state, last_P = (self.history[-1].state, self.history[-1].P) if len(self.history) > 0 else (None, None)
        flag, new_state, new_P = linear_kalman(measurements, last_state, last_P)
        if flag:
            self.history.append(Entry(time_stamp, new_state, new_P, kwargs['GDOP'], kwargs['source']))


class FederatedKalmanFilter:
    history: list[Entry]
    filters: dict[Source, LocalKalmanFilter]
    last_time_stamp: TimeStamp or None

    evaluation_funcs = [
        lambda lkf: np.trace(lkf.last.P),
        lambda lkf: lkf.last.GDOP,
        lambda lkf: norm(lkf.get_derivative()),
    ]
    weights = [0, 1, 0]

    def __init__(self, used_sources):
        self.history = []
        self.filters = {source: LocalKalmanFilter() for source in used_sources}
        self.last_time_stamp = None

    def update(self, source, measurements, solve, time_stamp):
        self.filters[source].update(measurements, time_stamp, source=source, **solve)
        self.last_time_stamp = time_stamp

    def choose_next(self):
        if self.last_time_stamp is None:
            return
        filterable = [lkf for lkf in self.filters.values() if lkf and lkf.last.stamp == self.last_time_stamp]
        eps = [min(filterable, key=func) for func in self.evaluation_funcs]

        selected_options = {self.weights[i] * self.evaluation_funcs[i](lkf): lkf for i, lkf in enumerate(eps)}
        selected_lkf = min(selected_options.items(), key=lambda item: item[0])
        self.history.append(selected_lkf[1].get_last())



