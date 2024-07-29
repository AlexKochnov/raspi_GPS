import os
import pickle
import threading
import traceback
import numpy as np
from numpy.linalg import inv, norm
import pandas as pd
from datetime import datetime
import concurrent.futures

import pyubx2

import Constants
import Minimizing
import Settings
import Transformations
# from Messages import *
# import Messages
import UBXMessages
from NavTaskUtils import make_ae_nav_data
from TimeStamp import TimeStamp
from GNSS import GNSS, get_GNSS_len
from StorageColumnsLord import StorageColumnsLord as SCL

# TODO: подлатать
pd.set_option('future.no_silent_downcasting', True)


# import warnings
# warnings.filterwarnings("error")


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


optimize_methods = {
    'LM': Minimizing.solve_navigation_task_LevMar,
    'SQP': Minimizing.solve_navigation_task_SLSQP,
    'TC': Minimizing.solve_navigation_task_TC,
    'DB': Minimizing.solve_navigation_task_DogBox,
    'TRF': Minimizing.solve_navigation_task_TRF,
}


# class TimeoutException(Exception):
#     pass
#
# def timer_handler():
#     raise TimeoutException()

def calc_receiver_coordinates(data_table: pd.DataFrame, solve_table: pd.DataFrame, stamp: TimeStamp):
    sats = data_table.copy()
    sats = sats[(sats.nav_score > 0) & (sats.coord_score > 0) &
                (stamp - sats.xyz_stamp < 10) & (stamp - sats.pr_stamp < 10)]
    sats = sats.dropna()
    sats['score'] = sats.coord_score + sats.nav_score
    sats = sats.sort_values(by='score', ascending=False).head(Settings.MinimizingSatellitesCount)
    # data = [(row.X, row.Y, row.Z, row.prMes) for row in sats.iterrows()]
    data = [(row['X'], row['Y'], row['Z'], row['prMes']) for index, row in sats.iterrows()]
    SOLVE = {'calc_stamp': stamp, 'sat_count': len(data)}
    if len(data) >= Settings.MinimumMinimizingSatellitesCount:
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
                res = func(data)
                lla = Transformations.ecef2lla(*res.x[:-1])
                solve = {
                    'X': res.x[0], 'Y': res.x[1], 'Z': res.x[2], 'cdt': res.x[3], 'dt': res.x[3] / Constants.c,
                    'lat': lla[0], 'lon': lla[1], 'alt': lla[2],
                    'fval': sum(res.fun ** 2) if method in ['LM', 'TRF', 'DB'] else res.fun,
                    'success': res.success,
                    'error': np.linalg.norm(np.array(res.x[:3]) - np.array(Constants.ECEF)),
                    'calc_time': (datetime.now(tz=Constants.tz_utc) - t).total_seconds(),
                }
                # solve = {f'{name}_{key}': value for key, value in solve.items()}
                return solve | {'method': method}
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future = executor.submit(get_solve_line)
                try:
                    SOLVE |= future.result(timeout=Settings.max_calc_time)
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


SFRBX_DATA_PATH = 'sfrbx_data.plk'


class DynamicObject(object):
    pass

def get_DynStorage(storage):
    Dyn = DynamicObject()
    Dyn.ephemeris_parameters1 = storage.ephemeris_parameters1
    Dyn.almanac_parameters1 = storage.almanac_parameters1
    Dyn.navigation_parameters1 = storage.navigation_parameters1
    Dyn.ephemeris_data1 = storage.ephemeris_data1
    Dyn.almanac_data1 = storage.almanac_data1

    Dyn.ephemeris_solves1 = storage.ephemeris_solves
    Dyn.almanac_solves1 = storage.almanac_solves
    Dyn.general_data1 = storage.general_data
    Dyn.FK_coordinates_xyz = storage.FK_coordinates_xyz
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
        return [np.nan, np.nan, np.nan], np.nan
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
    print('MATRIX:', P_k1k1)
    return list(X_k1k1), np.matrix(P_k1k1)


def FK_filter(storage):
    coord_names = ['X', 'Y', 'Z']

    if len(storage.general_data) < 1 or len(storage.ephemeris_solves) < 1 or len(storage.almanac_solves) < 1:
        return
    a=0
    lines = [
        #TODO: поправить синхронизацию и добавить проверку времени
        # storage.general_data[storage.general_data['receiving_stamp'] == storage.time_stamp][coord_names].iloc[-1],
        # storage.ephemeris_solves[storage.ephemeris_solves['calc_stamp'] == storage.time_stamp][coord_names].iloc[-1],
        # storage.almanac_solves[storage.almanac_solves['calc_stamp'] == storage.time_stamp][coord_names].iloc[-1],
        storage.general_data.iloc[-1][['receiving_stamp'] + [f'ecef{name}' for name in coord_names]],
        storage.ephemeris_solves.iloc[-1][['calc_stamp'] + coord_names],
        storage.almanac_solves.iloc[-1][['calc_stamp'] + coord_names],
    ]
    # def get_last_line(table):
    #     if len(table):
    #         return table.iloc[-1]
    #     return None
    if len(storage.FK_coordinates_xyz) < 1:
        NoneData = True
    else:
        NoneData = False
    n = len(storage.FK_coordinates_xyz)
    # storage.FK_coordinates_xyz.loc[n] = {'receiving_stamp': storage.time_stamp} | {f'P_{s}': None for s in ['rec', 'eph', 'alm']}
    data = {'receiving_stamp': storage.time_stamp}
    try:
        for i, source in enumerate(['rec', 'eph', 'alm']):
            cols = [f'{name}_{source}' for name in coord_names]
            if NoneData is False:
                line = storage.FK_coordinates_xyz.iloc[-1, :]
                last_xyz = line[cols].to_list()
                last_P = line[f'P_{source}']
            else:
                last_xyz, last_P = None, None
            res_xyz, res_P = linear_kalman(lines[i][1:].to_list(), last_xyz, last_P)
            # storage.FK_coordinates_xyz.loc[n, cols[0]] = res_xyz[0]
            # storage.FK_coordinates_xyz.loc[n, cols[1]] = res_xyz[1]
            # storage.FK_coordinates_xyz.loc[n, cols[2]] = res_xyz[2]
            # storage.FK_coordinates_xyz.loc[n, f'P_{source}'] = res_P
            data |= {cols[0]: res_xyz[0], cols[1]: res_xyz[1], cols[2]: res_xyz[2],
                     f'P_{source}': res_P, f'normP_{source}': np.linalg.norm(res_P)}
            #        # {'receiving_stamp': lines[i][0]}
            # storage.FK_coordinates_xyz.loc[n] = data
            # print("LINE:", storage.FK_coordinates_xyz.iloc[-1])
            # storage.FK_coordinates_xyz.iloc[-1, storage.FK_coordinates_xyz.columns.get_loc(coord_cols)] = res_xyz
            # storage.FK_coordinates_xyz.iloc[-1, storage.FK_coordinates_xyz.columns.get_loc(f'P_{source}')] = res_P
        storage.FK_coordinates_xyz.loc[n] = data
    except Exception as e:
        print(e)
        print(traceback.format_exc())
        a=0



class Storage:
    week: int = None
    TOW: int = None
    iTOW: int = None
    last_iTOW: int = None
    time_stamp: TimeStamp = None

    other_data = dict()

    leapS = 18

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
        self.general_data = pd.DataFrame(columns=[
            'receiving_stamp', 'week', 'iTOW',
            'ecefX', 'ecefY', 'ecefZ', 'pAcc',
            'fTOW', 'tAcc', 'leapS', 'towValid', 'weekValid', 'leapSValid',
            'tau_c', 'tau_GPS', 'N4', 'NA', 'B1', 'B2', 'KP',
        ])
        self.FK_coordinates_xyz = pd.DataFrame(
            columns=['receiving_stamp'] +
                    [f'{coord}_{source}' for source in ['rec', 'eph', 'alm'] for coord in ['X', 'Y', 'Z', 'P', 'normP']]
        )
        # self.FK_coordinates_xyz[[f'P_{source}' for source in ['rec', 'eph', 'alm']]].astype(object)

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

        if Settings.GUI_ON:
            self.ADD_GUI_TABLES()

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

    def update_UBX(self, message):
        self.time_stamp: TimeStamp = message.receiving_stamp
        if isinstance(message, UBXMessages.AID_ALM | UBXMessages.AID_EPH):
            table = self.almanac_parameters if isinstance(message, UBXMessages.AID_ALM) else self.ephemeris_parameters
            data = message.stamp | {'receiving_stamp': message.receiving_stamp}
            if message.data:
                table.update(create_index_table([message.data | {'is_old': False, 'exist': True} | data]))
            else:
                table.update(create_index_table([{'is_old': True} | data]))
        elif isinstance(message, UBXMessages.NAV_SAT | UBXMessages.NAV_ORB | UBXMessages.RXM_RAWX | \
                                 UBXMessages.RXM_MEASX):
            self.update_param(message.data, 'iTOW', 'week')
            if message.satellites:
                self.navigation_parameters.update(create_index_table(message.satellites))
                self.check_nav_time()
            if isinstance(message, UBXMessages.RXM_RAWX):
                self.calc_navigation_task()
                FK_filter(self)
                self.flush_flag = True
        elif isinstance(message, UBXMessages.NAV_TIMEGPS | UBXMessages.NAV_POSECEF | UBXMessages.NAV_VELECEF):
            assert isinstance(message.data, dict)
            self.update_param(message.data, 'iTOW', 'week', 'leapS')
            if 'leapS' in message.data.keys():
                Constants.leapS = message.data['leapS']
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
        if self.counter % 30 == 0:
            pass
            self.navigation_parameters.to_csv('nav_params.csv')
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
        if not len(ind):
            data['receiving_stamp'] = stamp
            table.loc[len(table)] = data
            return
        ind = ind[0]
        for key, value in data.items():
            if key in table.columns:
                table.at[ind, key] = value


    def calc_navigation_task(self):
        # try:
        self.ephemeris_data, self.almanac_data = \
            make_ae_nav_data(self.navigation_parameters, self.ephemeris_parameters, self.almanac_parameters,
                             self.time_stamp)
        calc_receiver_coordinates(self.ephemeris_data, self.ephemeris_solves, self.time_stamp)
        calc_receiver_coordinates(self.almanac_data, self.almanac_solves, self.time_stamp)
        # except Exception as e:
        #     print(e)
        #     print(tr := traceback.format_exc())
        #     a = 0

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
             'health', 'qualityInd', 'prValid', 'mpathIndic', 'orbitSourse', 'visibility', 'svUsed']]
        self.navigation_parameters1.rename(columns={'receiving_stamp': 'receiving'})
        self.almanac_parameters1 = self.almanac_parameters.apply(row_table_cleaning, axis=1, result_type='expand')
        self.almanac_data1 = self.almanac_data.apply(row_table_cleaning, axis=1, result_type='expand')
        # self.almanac_solves1 = self.almanac_solves.copy()
        self.ephemeris_parameters1 = self.ephemeris_parameters.apply(row_table_cleaning, axis=1, result_type='expand')
        self.ephemeris_data1 = self.ephemeris_data.apply(row_table_cleaning, axis=1, result_type='expand')
        # self.ephemeris_solves1 = self.ephemeris_solves.copy()
        # self.general_data1 = self.general_data.copy()

        self.DynStorage = get_DynStorage(self)


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
                                                       'omega_n', 'ln'])
        self.SFRBX_GLONASS_eph = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GLONASS} for j in range(1, 25)],
                                              columns=['svId', 'gnssId', 'receiving_stamp', 'sfN1', 'sfN2', 'sfN3',
                                                       'sfN4', 'tk', 'tb', 'tau', 'dTau', 'gamma', 'N_T', 'F_T', 'E',
                                                       'n', 'x', 'y', 'z', 'dx', 'dy', 'dz', 'ddx', 'ddy', 'ddz',
                                                       'ln', 'P', 'P1', 'P2', 'P3', 'P4', 'Bn', 'M'])
        # 'B1', 'B2', 'KP', 'tau_c', 'tau_GPS', 'N4', 'NA'
        self.SFRBX_GLONASS_data = dict()
