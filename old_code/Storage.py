import os
import pickle
import traceback
import numpy as np
from numpy.linalg import inv
import pandas as pd
from datetime import datetime
import concurrent.futures

import pyubx2

from Storage import Minimizing
from Utils import Settings, Constants, Transformations
# from Messages import *
# import Messages
from Messages import NMEAMessages, UBXMessages
from NavTaskUtils import make_ae_nav_data
from Utils.TimeStamp import TimeStamp
from Utils.GNSS import GNSS, get_GNSS_len
from StorageColumnsLord import StorageColumnsLord as SCL
from Utils.Transformations import lla2ecef

# TODO: подлатать
pd.set_option('future.no_silent_downcasting', True)


# import warnings
# warnings.filterwarnings("error")

# class TrackingDataFrame(pd.DataFrame):
#     def __getitem__(self, key):
#         print(traceback.format_exc())
#         print(f"Accessing column/row: {key}")
#         return super().__getitem__(key)
#
#     def __setitem__(self, key, value):
#         print(traceback.format_exc())
#         print(f"Setting value for column/row: {key}")
#         return super().__setitem__(key, value)
#
#     def __getattr__(self, name):
#         print(traceback.format_exc())
#         print(f"Accessing attribute: {name}")
#         return super().__getattr__(name)
#
#     def __setattr__(self, name, value):
#         print(traceback.format_exc())
#         print(f"Setting attribute: {name}")
#         super().__setattr__(name, value)
#
#     def __call__(self, *args, **kwargs):
#         print(traceback.format_exc())
#         print(f"Calling DataFrame method with arguments: {args}, {kwargs}")
#         return super().__call__(*args, **kwargs)


def calc_pseu_RMS_error(index):
    if pd.isna(index):
        return np.nan
    index = int(index)
    y = (index >> 3) & 0x7
    x = (index) & 0x7
    RMS = 0.5 * (1 + x / 8) * 2 ** y
    return RMS


def update_table_line(table, data, svId, gnssId, receiving_stamp, TOW_type='receiving_stamp'):
    ind = table.index[(table.svId == svId) & (table.gnssId == gnssId)]
    # print(type(ind))
    if not len(ind):
        data['svId'] = svId
        data['gnssId'] = gnssId
        # TODO добавить другие ГНСС системы

        # table.loc[len(table)] = data
        # table = pd.concat([table, pd.DataFrame([data])], ignore_index=True)
        return
    ind = ind[0]
    if receiving_stamp:
        table.at[ind, TOW_type] = receiving_stamp
    for key, value in data.items():
        if key in table.columns:
            table.at[ind, key] = type(table.at[ind, key])(value)  # value


def custom_min(*args):
    arr = [x for x in args if x is not None and pd.notna(x)]
    if arr:
        return min(arr)
    return np.nan




# class TimeoutException(Exception):
#     pass
#
# def timer_handler():
#     raise TimeoutException()

def calc_GDOP(satellites, position):
    X, Y, Z, cdt = position
    def calc_row(sat):
        dx, dy, dz = sat.X - X, sat.Y - Y, sat.Z - Z
        # R = np.sqrt(dx**2 + dy**2 + dz**2)
        R = sat.prDist
        return np.array([dx/R, dy/R, dz/R, 1])
    G = np.array([calc_row(sat) for i, sat in satellites.iterrows()])
    Qinv = G.T @ G
    Q = np.linalg.inv(Qinv)
    # Gx, Gy, Gz, Gt = (Q[i, i] for i in range(4))
    # PDOP = np.sqrt(Gx + Gy + Gz)
    # TDOP = np.sqrt(Gt)
    return float(np.sqrt(np.trace(Q)))

optimize_methods = {
    'LM': Minimizing.solve_navigation_task_LevMar,
    'SQP': Minimizing.solve_navigation_task_SLSQP,
    'TC': Minimizing.solve_navigation_task_TC,
    'DB': Minimizing.solve_navigation_task_DogBox,
    'TRF': Minimizing.solve_navigation_task_TRF,
}


def calc_receiver_coordinates(data_table: pd.DataFrame, solve_table: pd.DataFrame, stamp: TimeStamp, type):
    sats = data_table.copy()
    pd.set_option('display.max_colwidth', None)
    # print(sats)
    sats = sats.dropna()
    # if len(sats):
    #     print(max(stamp - sats.xyz_stamp), max(stamp - sats.pr_stamp))
    #TODO: вернуть проверку на время
    sats = sats[(sats.nav_score > 15) & (sats.coord_score > 0)]# &
                #(stamp - sats.xyz_stamp < 10) & (stamp - sats.pr_stamp < 10)]
    # head = 4 + np.random.randint(4) #
    sats = sats.sort_values(by='nav_score', ascending=False).head(Settings.MaximumMinimizingSatellitesCount)
    if len(sats) > 5 and type == 'eph':
        sats.to_csv(f'data/{type}_{len(sats)}_{datetime.now().timestamp()}.csv')
    # print(stamp)
    # print(sats)
    data_table['used'] = data_table.apply(lambda row: row.svId in sats.svId.values, axis=1)
    # data = [(row.X, row.Y, row.Z, row.prMes) for row in sats.iterrows()]
    # data = [(row['X'], row['Y'], row['Z'], row['prMes']) for index, row in sats.iterrows()]
    # if np.random.randint(20) == 5:
    #     a=0
    SOLVE = {'calc_stamp': stamp, 'sat_count': len(sats), 'success': False}
    if len(sats) >= Settings.MinimumMinimizingSatellitesCount:

        for method, func in optimize_methods.items():
            # if name not in Settings.using_methods:
            if method != Settings.used_method:#(METHOD if METHOD else Settings.used_method):
                continue
            # signal.signal(signal.SIGALRM, handler)
            # signal.alarm(Settings.max_calc_time)
            # timer = threading.Timer(Settings.max_calc_time, timer_handler)
            # timer.start()
            # try:
            def get_solve_line():
                t = datetime.now(tz=Constants.tz_utc)
                # res = func(sats[['X', 'Y', 'Z', 'prMes']])
                sats['prMesCorrected'] = sats['prMes'] + Constants.c * sats['af_dt']
                res = func(np.array(sats[['X', 'Y', 'Z', 'prMesCorrected']].values.T))
                lla = Transformations.ecef2lla(*res.x[:-1])
                solve = {
                    'X': res.x[0], 'Y': res.x[1], 'Z': res.x[2], 'cdt': res.x[3], 'dt': res.x[3] / Constants.c,
                    'min_point': res.x,
                    'lat': lla[0], 'lon': lla[1], 'alt': lla[2],
                    'fval': np.linalg.norm(res.fun) if method in ['LM', 'TRF', 'DB'] else res.fun,
                    'success': res.success,
                    'error': np.linalg.norm(np.array(res.x[:3]) - np.array(Constants.ECEF)),
                    'calc_time': (datetime.now(tz=Constants.tz_utc) - t).total_seconds(),
                }
                if method == 'TC':
                    a=0
                # solve = {f'{name}_{key}': value for key, value in solve.items()}
                sats['prDist'] = sats['prMesCorrected'] - res.x[-1]
                return solve | {'method': method} | {'GDOP': calc_GDOP(sats, res.x)}
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future = executor.submit(get_solve_line)
                try:
                    SOLVE |= future.result(timeout=Settings.max_calc_time)
                    if SOLVE['error'] < 1e4:
                        Settings.LastDtDelay = SOLVE['dt']
                except concurrent.futures.TimeoutError:
                    SOLVE |= {'method': method, 'calc_time': '>limit'}
                except Exception as e:
                    SOLVE |= {'method': method, 'calc_time': 'error'}
                # signal.alarm(0)
            # except TimeoutException:
            #     SOLVE |= {'method': method, 'calc_time': '>limit'}
            # except Exception as e:
            #     print(e)
            #     print(tr := traceback.format_exc())
            #     a = 0
            # finally:
                # timer.cancel()
    solve_table.loc[len(solve_table)] = SOLVE


def get_df_indexes(df):
    return df.apply(index_func, axis=1)


def index_func(row):
    return row.svId + GNSS(row.gnssId).value * 100


def create_table(columns: dict, init_sats_lines: pd.DataFrame = pd.DataFrame()):
    #TODO: вернуть типы
    # types = np.dtype(list(columns.items()))
    # table = pd.DataFrame(np.empty(len(init_sats_lines), dtype=types))
    table = pd.DataFrame(np.nan, index=init_sats_lines.index, columns=list(columns.keys()))

    if not init_sats_lines.empty:
        table.set_index(init_sats_lines.index, inplace=True)
        table.update(init_sats_lines)
    return table


def create_index_table(data):
    table = pd.DataFrame(data)
    table.set_index(get_df_indexes(table), inplace=True)
    return table


def make_init_df(*gnss_list: GNSS):
    def init_lines_to_df(lines: list[dict]):
        init_df = pd.DataFrame(lines).astype(SCL.stamp_columns)
        init_df.set_index(get_df_indexes(init_df), inplace=True)
        return init_df

    lines = []
    for gnss in gnss_list:
        lines += [{'svId': j + 1, 'gnssId': gnss} for j in range(get_GNSS_len(gnss))]
    return init_lines_to_df(lines)


SFRBX_DATA_PATH = 'sfrbx_data_2.plk'


class DynamicObject(object):
    pass


def get_DynStorage(storage):
    Dyn = DynamicObject()
    Dyn.ephemeris_parameters1 = storage.ephemeris_parameters1
    Dyn.almanac_parameters1 = storage.almanac_parameters1
    Dyn.navigation_parameters1 = storage.navigation_parameters1
    Dyn.ephemeris_data1 = storage.ephemeris_data1
    Dyn.almanac_data1 = storage.almanac_data1

    Dyn.ephemeris_solves1 = storage.ephemeris_solves.copy()
    Dyn.almanac_solves1 = storage.almanac_solves.copy()
    Dyn.general_data1 = storage.general_data.copy()

    Dyn.LFK_coordinates_xyz = storage.LFK_coordinates_xyz.copy()
    Dyn.LFK_coordinates_lla = storage.LFK_coordinates_lla.copy()
    Dyn.FFK_filtered = storage.FFK_filtered.copy()

    # try:
    #     tables = [table.rename(columns=lambda x: f'{x}_{name}' if x in ['X', 'Y', 'Z'] else x)
    #               for name, table in storage.filtered_tables_xyz.items()]
    #     Dyn.FK_coordinates_xyz = pd.merge(tables[0], tables[1], on='receiving_stamp')
    #     Dyn.FK_coordinates_xyz.merge(tables[2], on='receiving_stamp')
    #     # Dyn.FK_coordinates_lla = pd.merge(storage.filtered_tables_lla.items(), on='receiving_stamp',
    #     #                                   suffixes=[f'_{name}' for name in storage.filtered_tables_lla.keys()])
    #     tables = [table.rename(columns=lambda x: f'{x}_{name}' if x in ['lat', 'lon', 'alt'] else x)
    #               for name, table in storage.filtered_tables_xyz.items()]
    #     Dyn.FK_coordinates_lla = pd.merge(tables[0], tables[1], on='receiving_stamp')
    #     Dyn.FK_coordinates_lla.merge(tables[2], on='receiving_stamp')
    # except Exception as e:
    #     print(e)
    #     a=0

    Dyn.SFRBX_GLONASS_alm = storage.SFRBX_GLONASS_alm
    Dyn.SFRBX_GLONASS_eph = storage.SFRBX_GLONASS_eph
    return Dyn


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


def FK_filter1(storage):
    if len(storage.general_data) < 1 or len(storage.ephemeris_solves) < 1 or len(storage.almanac_solves) < 1:
        return
    types = ['xyz', 'lla']
    sources = ['rec', 'alm', 'eph']
    for type in types:
        for source in sources:
            # try:
            if source == 'rec':
                measurements_xyz = storage.general_data.iloc[-1][['receiving_stamp', 'ecefX', 'ecefY', 'ecefZ']].values.tolist()
            elif source == 'alm':
                measurements_xyz = storage.almanac_solves.iloc[-1][['calc_stamp', 'X', 'Y', 'Z']].values.tolist()
            else:
                measurements_xyz = storage.ephemeris_solves.iloc[-1][['calc_stamp', 'X', 'Y', 'Z']].values.tolist()
            stamp = measurements_xyz[0]

            if type == 'xyz':
                if len(storage.filtered_tables_xyz[source]) > 0:
                    last_filtered_values = storage.filtered_tables_xyz[source].iloc[-1][['X', 'Y', 'Z', 'P']].values.tolist()
                else:
                    last_filtered_values = None
                measurements = measurements_xyz[1:4]
            else:
                if len(storage.filtered_tables_lla[source]) > 0:
                    last_filtered_values = storage.filtered_tables_lla[source].iloc[-1][['lat', 'lon', 'alt', 'P']].values.tolist()
                else:
                    last_filtered_values = None
                measurements = Transformations.ecef2lla(*measurements_xyz[1:4])
            if last_filtered_values is None:
                last_coords, last_P = None, None
            else:
                last_coords, last_P = last_filtered_values[1:3], last_filtered_values[4]

            res_coords, res_P = linear_kalman(measurements, last_coords, last_P)

            if type == 'xyz':
                storage.filtered_tables_xyz[source].loc[len(storage.filtered_tables_xyz[source])] = \
                    {'receiving_stamp': stamp, 'X': res_coords[0], 'Y': res_coords[1], 'Z': res_coords[2],
                     'P': res_P, 'normP': np.linalg.norm(res_P)}
            else:
                storage.filtered_tables_xyz[source].loc[len(storage.filtered_tables_xyz[source])] = \
                    {'receiving_stamp': stamp, 'lat': res_coords[0], 'lon': res_coords[1], 'alt': res_coords[2],
                     'P': res_P, 'normP': np.linalg.norm(res_P)}
            # except Exception as e:
            #     print(e)
            #     print(measurements, last_coords, last_P)
            #     print
            #     a=0




    pass

def LFK(storage, xyz_flag=True):
    coord_names = ['X', 'Y', 'Z'] if xyz_flag else ['lat', 'lon', 'alt']
    FK_table = storage.LFK_coordinates_xyz if xyz_flag else storage.LFK_coordinates_lla

    if len(storage.general_data) < 1 or len(storage.ephemeris_solves) < 1 or len(storage.almanac_solves) < 1:
        return
    coords_xyz = ['X', 'Y', 'Z']
    lines = [
        #TODO: поправить синхронизацию и добавить проверку времени
        # storage.general_data[storage.general_data['receiving_stamp'] == storage.time_stamp][coord_names].iloc[-1],
        # storage.ephemeris_solves[storage.ephemeris_solves['calc_stamp'] == storage.time_stamp][coord_names].iloc[-1],
        # storage.almanac_solves[storage.almanac_solves['calc_stamp'] == storage.time_stamp][coord_names].iloc[-1],
        storage.general_data.iloc[-1][['receiving_stamp', 'GDOP'] + [f'ecef{name}' for name in coords_xyz]],
        storage.ephemeris_solves.iloc[-1][['calc_stamp', 'GDOP'] + coords_xyz],
        storage.almanac_solves.iloc[-1][['calc_stamp', 'GDOP'] + coords_xyz],
    ]
    # NoneData = True if len(FK_table) < 1 else False
    n = len(FK_table)
    # storage.FK_coordinates_xyz.loc[n] = {'receiving_stamp': storage.time_stamp} | {f'P_{s}': None for s in ['rec', 'eph', 'alm']}
    data = {'receiving_stamp': storage.time_stamp}
    # try:
    for i, source in enumerate(['rec', 'eph', 'alm']):
        cols = [f'{name}_{source}' for name in coord_names]
        # if NoneData is False:
        #     line = FK_table.iloc[-1, :]
        #     last_coords = line[cols].to_list()
        #     last_P = line[f'P_{source}']
        # else:
        #     last_coords, last_P = None, None
        last_coords, last_P = storage.last_filtered_coordinates['xyz' if xyz_flag else 'lla'][i]

        coords = lines[i][2:].to_list() if xyz_flag else Transformations.ecef2lla(*lines[i][2:].to_list())
        flag, res_coords, res_P = linear_kalman(coords, last_coords, last_P)
        # storage.FK_coordinates_xyz.loc[n, cols[0]] = res_xyz[0]
        # storage.FK_coordinates_xyz.loc[n, cols[1]] = res_xyz[1]
        # storage.FK_coordinates_xyz.loc[n, cols[2]] = res_xyz[2]
        # storage.FK_coordinates_xyz.loc[n, f'P_{source}'] = res_P
        data |= {cols[0]: res_coords[0], cols[1]: res_coords[1], cols[2]: res_coords[2],
                 f'P_{source}': res_P, f'normP_{source}': np.linalg.norm(res_P), f'GDOP_{source}': lines[i][1]}
        if flag:
            storage.last_filtered_coordinates['xyz' if xyz_flag else 'lla'][i] = [res_coords, res_P]
    FK_table.loc[n] = data
    # except Exception as e:
    #     print(e)
    #     print(traceback.format_exc())
    #     a=0


def FFK(storage, xyz_flag):
    if xyz_flag:
        table = storage.LFK_coordinates_xyz
        names = ['X', 'Y', 'Z', 'P']
    else:
        table = storage.LFK_coordinates_lla
        names = ['lat', 'lon', 'alt', 'P']
    if not len(table):
        return None
    sources = ['rec', 'alm', 'eph']

    def get_data_from_ind(ind):
        if len(table) >= ind:
            line = table.iloc[-ind]
            return line['receiving_stamp'], [line[[f'{elem}_{source}' for elem in names]] for source in sources]
        return None, None

    # по норме P
    tmp, datas = get_data_from_ind(1)
    ind, val = min(enumerate(datas), key=lambda row: np.trace(row[1].iloc[3]) if np.array(row[1].iloc[3]).ndim >= 2 else np.nan)
    # return tmp, sources[ind], val[:3].values, np.linalg.norm(val[3])

    # по производной
    _, datas2 = get_data_from_ind(2)
    if datas2:
        derivs = [[datas[i][f'{elem}_{source}'] - datas2[i][f'{elem}_{source}'] for elem in names[:-1]] for i, source in enumerate(sources)]
        ind, _ = min(enumerate(derivs), key=lambda row: np.linalg.norm(row[1]))
        val = datas[ind]

    return tmp, sources[ind], val[:3].values, np.linalg.norm(val[3])



def FFK_combo(storage):
    n = len(storage.FFK_filtered)
    # storage.FFK_filtered.loc[n] = {'receiving_stamp': storage.time_stamp}
    storage.FFK_filtered.loc[n] = {'receiving_stamp': storage.time_stamp}
    xyz_data = FFK(storage, True)
    if xyz_data is not None:
        ts, source, (x, y, z), val = xyz_data
        if len(storage.FFK_filtered.loc[storage.FFK_filtered.receiving_stamp == ts]):
            storage.FFK_filtered.loc[storage.FFK_filtered.receiving_stamp == ts,
                                 ['X', 'Y', 'Z', 'xyz_source', 'xyz_val']] = [x, y, z, source, val]
    lla_data = FFK(storage, False)
    if lla_data is not None:
        ts, source, (lat, lon, alt), val = lla_data
        if len(storage.FFK_filtered.loc[storage.FFK_filtered.receiving_stamp == ts]):
            storage.FFK_filtered.loc[storage.FFK_filtered.receiving_stamp == ts,
                                 ['lat', 'lon', 'alt', 'lla_source', 'lla_val']] = [lat, lon, alt, source, val]
    pass


class Storage:
    week: int = None
    TOW: int = None
    iTOW: int = None
    last_iTOW: int = None
    time_stamp: TimeStamp = None

    other_data = dict()

    leapS = 18
    rcvTow = 0
    fTOW = 0

    flush_flag = False
    DynStorage = None

    def __init__(self):
        self.counter = 0

        init0 = make_init_df(GNSS.GPS)
        init06 = make_init_df(GNSS.GPS, GNSS.GLONASS)

        self.ephemeris_parameters = create_table(SCL.full_eph_columns, init0)
        self.almanac_parameters = create_table(SCL.full_alm_columns, init0)
        self.navigation_parameters = create_table(SCL.full_nav_columns, init06)  # GP + GL
        self.ephemeris_data = create_table(SCL.data_columns, init0)
        self.almanac_data = create_table(SCL.data_columns, init0)
        self.ephemeris_solves = create_table(SCL.full_solves_columns)
        self.almanac_solves = create_table(SCL.full_solves_columns)

        # self.general_data = TrackingDataFrame(columns=[
        self.general_data = pd.DataFrame(columns=[
            'receiving_stamp', 'week', 'iTOW',
            'rcvTow', 'clkB', 'clkD',
            'ecefX', 'ecefY', 'ecefZ', 'pAcc', 'GDOP',
            'fTOW', 'tAcc', 'leapS', 'towValid', 'weekValid', 'leapSValid',
            'tau_c', 'tau_GPS', 'N4', 'NA', 'B1', 'B2', 'KP',
        ])
        self.LFK_coordinates_xyz = pd.DataFrame(
            columns=['receiving_stamp'] +
                    [f'{coord}_{source}' for source in ['rec', 'eph', 'alm'] for coord in ['X', 'Y', 'Z', 'P', 'normP', 'GDOP']]
        )
        self.LFK_coordinates_lla = pd.DataFrame(
            columns=['receiving_stamp'] +
                    [f'{coord}_{source}' for source in ['rec', 'eph', 'alm'] for coord in ['lat', 'lon', 'alt', 'P', 'normP', 'GDOP']]
        )
        # self.FK_coordinates_xyz[[f'P_{source}' for source in ['rec', 'eph', 'alm']]].astype(object)

        self.last_filtered_coordinates = {
            'xyz': [[None, None]] * 3,
            'lla': [[None, None]] * 3,
        }

        self.FFK_filtered = pd.DataFrame(
            columns=['receiving_stamp',
                     'X', 'Y', 'Z', 'xyz_source', 'xyz_val',
                     'lat', 'lon', 'alt', 'lla_source', 'lla_val']
        )

        # self.filtered_tables_xyz = dict()
        # self.filtered_tables_lla = dict()
        # sources = ['rec', 'alm', 'eph']
        # for source in sources:
        #     self.filtered_tables_xyz[source] = pd.DataFrame(
        #         columns=['receiving_stamp', 'X', 'Y', 'Z', 'P', 'normP'])
        #     self.filtered_tables_lla[source] = pd.DataFrame(
        #         columns=['receiving_stamp', 'lat', 'lon', 'alt', 'P', 'normP'])

        # TODO: integrate and delete
        if os.path.exists(SFRBX_DATA_PATH):
            with open(SFRBX_DATA_PATH, 'rb') as file:
                self.SFRBX_GPS_eph = pickle.load(file)
                self.SFRBX_GPS_alm = pickle.load(file)
                self.SFRBX_GLONASS_eph = pickle.load(file)
                self.SFRBX_GLONASS_alm = pickle.load(file)
                self.SFRBX_GPS_data = pickle.load(file)
                self.SFRBX_GLONASS_data = pickle.load(file)
        else:
            self.make_SFRBX_data()

    ### Обновление данных при приходе нового сообщения ###

    def update(self, message):
        self.counter += 1
        if not message:
            return
        if isinstance(message, pyubx2.ubxmessage.UBXMessage):
            return
        if isinstance(message, UBXMessages.UbxMessage):
            self.update_UBX(message)
        if isinstance(message, NMEAMessages.NmeaMessage):
            self.update_NMEA(message)

        if Settings.GUI_ON:
            self.ADD_GUI_TABLES()


        if self.counter % 100 == 0:
            match self.counter % 1000:
                case 100: self.general_data.to_csv(f'general_data_{Settings.START_ID}.csv')
                case 200: self.almanac_solves.to_csv(f'almanac_solves{Settings.START_ID}.csv')
                case 300: self.ephemeris_solves.to_csv(f'ephemeris_solves{Settings.START_ID}.csv')
                case 400: self.LFK_coordinates_xyz.to_csv(f'LFK_coordinates_xyz{Settings.START_ID}.csv')
                case 500: self.LFK_coordinates_lla.to_csv(f'LFK_coordinates_lla{Settings.START_ID}.csv')
                case 600: self.FFK_filtered.to_csv(f'FFK_filtered{Settings.START_ID}.csv')
        # self.counter += 1


        if self.iTOW:
            self.TOW = self.iTOW // 1000
        if self.iTOW != self.last_iTOW:
            self.last_iTOW = self.iTOW
            return True
        return False

    def update_param(self, data, *attrs):
        for attr in attrs:
            if attr in data.keys() and hasattr(self, attr):
                setattr(self, attr, data[attr])

    def check_nav_time(self):
        self.navigation_parameters['receiving_stamp'] = self.navigation_parameters.apply(
            lambda row: custom_min(row['NAV_ORB_stamp'], row['NAV_SAT_stamp'], row['RXM_RAWX_stamp']), axis=1)

    def update_NMEA(self, message):
        # if self.time_stamp is None:
        #     self.time_stamp: TimeStamp = message.receiving_stamp

        if isinstance(message, NMEAMessages.PSTMTS):
            if message.data:
                self.navigation_parameters.update(create_index_table([
                    message.data | {'prRMSer': 0, 'prRes': 0, 'prStedv': 0, 'rcvTOW': self.time_stamp.TOW,
                                    'qualityInd': 5, 'visibility': 3, 'prValid': True, 'health': 1,
                                    'almUsability': 1, 'almAvail': True, 'almSource': 1,
                                    'ephUsability': 1, 'ephAvail': True, 'ephSource': 1,
                                    }]))
        elif isinstance(message, NMEAMessages.GGA):
            LLA = (message.data['lat'], message.data['lon'], message.data['alt'])
            XYZ = lla2ecef(*LLA)
            # print(LLA)
            # print(XYZ)
            # a=0
            Constants.ECEF = XYZ
            self.update_general_data({'ecefX': XYZ[0], 'ecefY': XYZ[1], 'ecefZ': XYZ[2]}, self.time_stamp)
        elif isinstance(message, NMEAMessages.RMC):
            if self.time_stamp != message.receiving_stamp and self.time_stamp is not None:
                self.calc_navigation_task()
                # FK_filter1(self)
                LFK(self)
                LFK(self, xyz_flag=False)
                FFK_combo(self)
                self.flush_flag = True

            # новый цикл
            self.time_stamp = message.receiving_stamp
            self.rcvTow = self.time_stamp.TOW
            self.navigation_parameters['prMes'] = np.nan # чтобы учесть скачки задержек
            a=0
        elif isinstance(message, NMEAMessages.PSTMALMANAC):
            # try:
            if message.data:
                self.almanac_parameters.update(create_index_table([
                    message.data | {'is_old': False, 'exist': True, 'receiving_stamp': self.time_stamp, 'Data_ID': -1}]))
                # self.almanac_parameters.update([message.data])
            # except:
            #     a=0
        elif isinstance(message, NMEAMessages.PSTMEPHEM):
            if message.data:
                self.ephemeris_parameters.update(create_index_table([
                    message.data | {'is_old': False, 'exist': True, 'receiving_stamp': self.time_stamp}]))



    def update_UBX(self, message):
        # self.flush_flag = True

        if self.time_stamp is None:
            self.time_stamp: TimeStamp = message.receiving_stamp

        # print(message.receiving_stamp.dt)
        if isinstance(message, UBXMessages.AID_ALM | UBXMessages.AID_EPH):
            table = self.almanac_parameters if isinstance(message, UBXMessages.AID_ALM) else self.ephemeris_parameters
            data = message.stamp | {'receiving_stamp': message.receiving_stamp}
            if message.data:
                table.update(create_index_table([message.data | {'is_old': False, 'exist': True} | data]))
            else:
                table.update(create_index_table([{'is_old': True} | data]))
        elif isinstance(message, UBXMessages.NAV_SAT | UBXMessages.NAV_ORB | UBXMessages.RXM_RAWX | \
                                 UBXMessages.RXM_MEASX | UBXMessages.NAV_DOP):
            self.update_param(message.data, 'iTOW', 'week', 'gDOP')
            if isinstance(message, UBXMessages.NAV_DOP):
                pass
                if len(self.general_data):
                    self.general_data.at[self.general_data.index[-1], 'GDOP'] = message.data['gDOP']
            if isinstance(message, UBXMessages.RXM_RAWX):
                for i in range(len(message.satellites)):
                    message.satellites[i]['rcvTOW'] = \
                        TimeStamp(week=self.week, TOW=message.data['rcvTow'] + self.fTOW * 1e-9)
            if message.satellites:
                #TODO: поправить
                try:
                    self.navigation_parameters.update(create_index_table(message.satellites))
                except:
                    pass
                self.check_nav_time()
            if isinstance(message, UBXMessages.RXM_RAWX):
                self.update_param(message.data, 'rcvTow')
                # self.time_stamp.TOW = message.data['rcvTow']
                # self.update_general_data({'rcvTow': message.data['rcvTow']}, message.receiving_stamp)
                # self.general_data.iloc[-1].rcvTow = message.data['rcvTow']
                # try:
                if len(self.general_data):
                    self.general_data.at[self.general_data.index[-1], 'rcvTow'] = message.data['rcvTow']
                # except Exception as e:
                #     print(e)
                #     print(traceback.format_exc())

                self.calc_navigation_task()
                # FK_filter1(self)
                LFK(self)
                LFK(self, xyz_flag=False)
                FFK_combo(self)
                self.flush_flag = True
        # elif isinstance(message, UBXMessages.NAV_CLOCK):
        #     self.general_data.at[self.general_data.index[-1], 'clkB'] = message.data['clkB']
        #     self.general_data.at[self.general_data.index[-1], 'clkD'] = message.data['clkD']
        elif isinstance(message, UBXMessages.NAV_TIMEGPS | UBXMessages.NAV_POSECEF | UBXMessages.NAV_VELECEF | \
                                 UBXMessages.NAV_CLOCK):
            assert isinstance(message.data, dict)
            self.update_param(message.data, 'iTOW', 'week', 'leapS', 'fTOW')
            if isinstance(message, UBXMessages.NAV_TIMEGPS):
                Constants.leapS = message.data['leapS']
                self.time_stamp = TimeStamp(message.data['iTOW']/1000, message.data['week'])
            if isinstance(message, UBXMessages.NAV_POSECEF):
                Constants.ECEF = tuple(message.data[f'ecef{sym}'] for sym in 'XYZ')
                # a=0
            self.other_data[message.__class__.__name__.lower()] = message.data
            self.update_general_data(message.data, message.receiving_stamp)
        elif isinstance(message, UBXMessages.RXM_SFRBX):
            assert isinstance(message.data, dict)
            try:  # TODO: править очень жестко
                if message.data['gnssId'] == GNSS.GPS:
                    if message.data['id'] == -1:
                        self.SFRBX_GPS_data.update(message.signal)
                        # TODO: add configs and health and ion processing
                    elif message.data['id'] == 0:
                        # TODO: update_line() -> df.update()
                        update_table_line(self.SFRBX_GPS_eph, message.signal,
                                          message.data['svId'], GNSS.GPS, None)
                    else:
                        update_table_line(self.SFRBX_GPS_alm, message.signal,
                                          message.signal['SV_ID'], GNSS.GPS, None)
                elif message.data['gnssId'] == GNSS.GLONASS:
                    if message.data['id'] == -1:
                        if 'NA' in message.signal.keys():
                            self.SFRBX_GLONASS_alm['NA'] = message.signal['NA']
                        self.SFRBX_GLONASS_data.update(message.signal)
                        self.update_general_data(message.signal, message.receiving_stamp)
                    elif message.data['id'] == 0:
                        update_table_line(self.SFRBX_GLONASS_eph, message.signal | \
                                          {f'sfN{message.data["StringN"]}': message.data['superframeN']},
                                          message.data['svId'], GNSS.GLONASS, message.receiving_stamp)
                    else:
                        update_table_line(self.SFRBX_GLONASS_alm, message.signal | \
                                          {f'sfN{message.data["StringN"] % 2}': message.data['superframeN']},
                                          message.data['id'], GNSS.GLONASS, message.receiving_stamp)
            except Exception as e:
                print(e)
                print(traceback.format_exc())
                a = 0
        # print(f'flush_flag: {self.flush_flag}')
        # if self.counter % 100 == 0 and False:
        #     match self.counter % 1000:
        #         case 100: self.general_data.to_csv(f'general_data_{Settings.START_ID}.csv')
        #         case 200: self.almanac_solves.to_csv(f'almanac_solves{Settings.START_ID}.csv')
        #         case 300: self.ephemeris_solves.to_csv(f'ephemeris_solves{Settings.START_ID}.csv')
        #         case 400: self.LFK_coordinates_xyz.to_csv(f'LFK_coordinates_xyz{Settings.START_ID}.csv')
        #         case 500: self.LFK_coordinates_lla.to_csv(f'LFK_coordinates_lla{Settings.START_ID}.csv')
        #         case 600: self.FFK_filtered.to_csv(f'FFK_filtered{Settings.START_ID}.csv')
            # self.navigation_parameters.to_csv('nav_params.csv')
            # self.general_data.to_csv('general_data.csv')
            # self.ephemeris_solves.to_csv('eph_solves.csv')
            # self.ephemeris_data.to_csv(f'epd_data{self.counter}.csv')
            # self.almanac_data.to_csv(f'alm_data{self.counter}.csv')
        if self.counter % 100 == 50:
            with open(SFRBX_DATA_PATH, 'wb') as file:
                pickle.dump(self.SFRBX_GPS_eph, file)
                pickle.dump(self.SFRBX_GPS_alm, file)
                pickle.dump(self.SFRBX_GLONASS_eph, file)
                pickle.dump(self.SFRBX_GLONASS_alm, file)
                pickle.dump(self.SFRBX_GPS_data, file)
                pickle.dump(self.SFRBX_GLONASS_data, file)

    def update_general_data(self, data: dict, stamp: TimeStamp):
        table = self.general_data
        ind = table.index[table.receiving_stamp == stamp]

        #TODO: вернуть
        if not len(ind):
            data['receiving_stamp'] = stamp
            table.loc[len(table)] = data
            # ind = table.index[table.receiving_stamp == stamp]
            return
        ind = ind[0]
        for key, value in data.items():
            if key in table.columns:
                table.at[ind, key] = value

    def calc_navigation_task(self):
        self.ephemeris_data, self.almanac_data = \
            make_ae_nav_data(self.navigation_parameters, self.ephemeris_parameters, self.almanac_parameters,
                             self.time_stamp, self.rcvTow)#self.time_stamp.TOW)#
                             #self.rcvTow + self.fTOW * 1e-9 + Settings.LastDtDelay)
        # print(self.rcvTow, Settings.LastDtDelay, self.rcvTow - Settings.LastDtDelay)
        # print(self.rcvTow + self.fTOW * 1e-9 + Settings.LastDtDelay, self.rcvTow, Settings.LastDtDelay)
        calc_receiver_coordinates(self.ephemeris_data, self.ephemeris_solves, self.time_stamp, 'eph')
        calc_receiver_coordinates(self.almanac_data, self.almanac_solves, self.time_stamp, 'alm')

    def ADD_GUI_TABLES(self):
        def row_table_cleaning(row):
            if any(pd.isna(row)):
                if 'is_old' in row.keys():
                    row.is_old = np.nan
                if 'receiving_stamp' in row.keys():
                    row.receiving_stamp = np.nan
                if 'xyz_stamp' in row.keys() and pd.isna(row.X):
                    row.xyz_stamp = np.nan
                    row.coord_score = np.nan
                if 'pr_stamp' in row.keys() and pd.isna(row.prMes):
                    row.pr_stamp = np.nan
                    row.nav_score = np.nan
            return row

        self.navigation_parameters1 = self.navigation_parameters[
            ['svId', 'gnssId', 'receiving_stamp', 'cno', 'prMes', 'prRes', 'prRMSer', 'prStedv', 'elev', 'azim',
             'ephUsability', 'ephSource', 'ephAvail', 'almUsability', 'almSource', 'almAvail',
             'health', 'qualityInd', 'prValid', 'mpathIndic', 'orbitSourse', 'visibility', 'svUsed',
             # 'wholeChips', 'fracChips', 'codePhase', 'intCodePhase',
                                      ]]
        self.navigation_parameters1.rename(columns={'receiving_stamp': 'receiving'})
        self.almanac_parameters1 = self.almanac_parameters.apply(row_table_cleaning, axis=1, result_type='expand')
        self.almanac_data1 = self.almanac_data.apply(row_table_cleaning, axis=1, result_type='expand')
        # self.almanac_solves1 = self.almanac_solves.copy()
        self.ephemeris_parameters1 = self.ephemeris_parameters.apply(row_table_cleaning, axis=1, result_type='expand')
        self.ephemeris_data1 = self.ephemeris_data.apply(row_table_cleaning, axis=1, result_type='expand')
        # self.ephemeris_solves1 = self.ephemeris_solves.copy()
        # self.general_data1 = self.general_data.copy()
        # self.FFK_filtered1

        self.DynStorage = get_DynStorage(self)
        a=0

    def make_SFRBX_data(self):
        self.SFRBX_GPS_alm = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)],
                                          columns=['svId', 'gnssId', 'Data_ID', 'SV_ID', 'e', 'Toa', 'delta_i', 'Wdot',
                                                   'health', 'sqrtA', 'W0', 'w', 'M0', 'af1', 'af0'])
        self.SFRBX_GPS_eph = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)],
                                          columns=['svId', 'gnssId', 'week', 'accuracy', 'health', 'IODC', 'Tgd', 'Toc',
                                                   'af2', 'af1', 'af0', 'IODE1', 'Crs', 'dn', 'M0', 'Cuc', 'e', 'Cus',
                                                   'sqrtA', 'Toe', 'Cic', 'W0', 'Cis', 'i0', 'Crc', 'w', 'Wdot',
                                                   'IODE2', 'IDOT'])
        self.SFRBX_GPS_data = dict()
        self.SFRBX_GLONASS_alm = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GLONASS} for j in range(1, 25)],
                                              columns=['svId', 'gnssId', 'receiving_stamp', 'sfN0', 'sfN1',
                                                       'n', 'lambda_n', 'delta_i_n', 'eps_n', 'M_n',
                                                       'tau_n', 'Cn', 'Hn', 't_lambda_n', 'delta_T_n', 'delta_T_dot_n',
                                                       'omega_n', 'ln', 'NA'])
        self.SFRBX_GLONASS_eph = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GLONASS} for j in range(1, 25)],
                                              columns=['svId', 'gnssId', 'receiving_stamp', 'sfN1', 'sfN2', 'sfN3',
                                                       'sfN4', 'tk', 'tb', 'tau', 'dTau', 'gamma', 'N_T', 'F_T', 'E',
                                                       'n', 'x', 'y', 'z', 'dx', 'dy', 'dz', 'ddx', 'ddy', 'ddz',
                                                       'ln', 'P', 'P1', 'P2', 'P3', 'P4', 'Bn', 'M'])
        # 'B1', 'B2', 'KP', 'tau_c', 'tau_GPS', 'N4', 'NA'
        self.SFRBX_GLONASS_data = dict()


