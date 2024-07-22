import math
import traceback
from enum import Enum
import numpy as np
import pandas as pd
from datetime import datetime, timedelta

import pyubx2

import Constants
import Minimizing
import Settings
import Transformations
# from Messages import *
# import Messages
import UBXMessages
from TimeStamp import TimeStamp
from UtilsMessages import GNSS
import SatellitesCoordinateCalculator as SCC

#TODO: подлатать
# pd.set_option('future.no_silent_downcasting', True)

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
            try:
                table.at[ind, key] = type(table.at[ind, key])(value) # value
            except FutureWarning as fw:
                print(fw)
                print(traceback.format_exc())
                a=0




def apply_func_get(table, func):
    result = {}
    for ind, row in table.iterrows():
        result[(row.svId, row.gnssId)] = func(row)
    return result


def update_table(table, datas):
    # t = datetime.now(tz=Constants.tz_utc)
    for stamp, data in datas.items():
        for key, value in data.items():
            table.loc[table.index[(table.svId == stamp[0]) & (table.gnssId == stamp[1])], key] = value
    # t2 = datetime.now(tz=Constants.tz_utc)
    # print(f'\t\tupdate: {t2 - t}')


def apply_func_set(table, func):
    update_table(table, apply_func_get(table, func))


def custom_min(*args):
    arr = [x for x in args if x is not None and pd.notna(x)]
    if arr:
        return min(arr)
    return np.nan


def get_row(table, svId, gnssId):
    return table[(table.svId == svId) & (table.gnssId == gnssId)]


def calc_receiver_coordinates(data_table: pd.DataFrame, solve_table: pd.DataFrame, stamp: TimeStamp):
    sats = data_table.copy()
    sats = sats[(sats.nav_score > 0) & (sats.coord_score > 0) &
                (stamp - sats.xyz_stamp < 10) & (stamp - sats.pr_stamp < 10)]
    sats = sats.dropna()
    sats['score'] = sats.coord_score + sats.nav_score
    sats = sats.sort_values(by='score', ascending=False).head(Settings.MinimizingSatellitesCount)
    # data = [(row.X, row.Y, row.Z, row.prMes) for row in sats.iterrows()]
    data = [(row['X'], row['Y'], row['Z'], row['prMes']) for index, row in sats.iterrows()]
    SOLVE = {'week': stamp.week, 'TOW': stamp.TOW, 'sat_count': len(data)}
    if len(data) >= Settings.MinimumMinimizingSatellitesCount:
        for name, func in [('LM', Minimizing.solve_navigation_task_LevMar),
                           ('SQP', Minimizing.solve_navigation_task_SLSQP)]:
            t = datetime.now(tz=Constants.tz_utc)
            res = func(data)
            solve = {
                'X': res.x[0], 'Y': res.x[1], 'Z': res.x[2], 'cdt': res.x[3], 'dt': res.x[3] / Constants.c,
                'fval': sum(res.fun) if name == 'LM' else res.fun,
                'success': res.success,
                'error': np.linalg.norm(np.array(res.x[:3]) - np.array(Constants.ECEF)),
                'calc_time': (datetime.now(tz=Constants.tz_utc) - t).total_seconds(),
            }
            solve = {f'{name}_{key}': value for key, value in solve.items()}
            SOLVE |= solve
    solve_table.loc[len(solve_table)] = SOLVE

def get_df_indexes(df):
    return df.apply(index_func, axis=1)

def index_func(row):
    return row.svId + GNSS(row.gnssId).value*100

def create_table(columns, init_sats_lines):
    types = np.dtype(list(columns.items()))
    table = pd.DataFrame(np.empty(len(init_sats_lines), dtype=types))
    table.set_index(init_sats_lines.index, inplace=True)
    table.update(init_sats_lines)
    return table

class Storage:
    NAV_ORB_columns = {'health': np.int8, 'visibility': np.int8, 'ephUsability': np.int8, 'ephSource': np.int8,
                       'almUsability': np.int8, 'almSource': np.int8}
    NAV_SAT_columns = {'cno': np.int8, 'elev': np.int8, 'azim': np.int16, 'prRes': np.float32, 'qualityInd': np.int8,
                       'svUsed': np.bool_, 'health': np.int8, 'diffCorr': np.bool_, 'smoothed': np.bool_,
                       'orbitSource': np.int8, 'ephAvail': np.bool_, 'almAvail': np.bool_}
    RXM_RAWX_columns = {'prMes': np.float64, 'cpMes': np.float64, 'doMes': np.float64,
                        'freqId': np.int8, 'locktime': np.uint16, 'cno': np.int8,
                        'prStedv': np.float32, 'cpStedv': np.float32, 'doStedv': np.float32,
                        'prValid': np.bool_, 'cpValid': np.bool_, 'halfCyc': np.bool_, 'subHalfCyc': np.bool_}
    RXM_SVSI_columns = {'azim': np.int16, 'elev': np.int8, 'ura': np.int8, 'healthy': np.bool_, 'ephVal': np.bool_,
                        'almVal': np.bool_, 'notAvail': np.bool_, 'almAge': np.int8, 'ephAge': np.int8}
    RXM_MEASX_columns = {'cno': np.int8, 'mpathIndic': np.int8, 'dopplerMS': np.float64, 'dopplerHz': np.float64,
                         'wholeChips': np.int16, 'fracChips': np.int16, 'codePhase': np.float64,
                         'intCodePhase': np.int8, 'pseuRangeRMSErr': np.int8}
    EPH_columns = {'week': np.int16, 'Toe': np.int32, 'Toc': np.int32, 'IODE1': np.int16, 'IODE2': np.int16,
                   'IODC': np.int16, 'IDOT': np.float64, 'Wdot': np.float64, 'Crs': np.float64, 'Crc': np.float64,
                   'Cus': np.float64, 'Cuc': np.float64, 'Cis': np.float64, 'Cic': np.float64, 'dn': np.float64,
                   'i0': np.float64, 'e': np.float64, 'sqrtA': np.float64, 'M0': np.float64, 'W0': np.float64,
                   'w': np.float64, 'Tgd': np.float64, 'af2': np.float64, 'af1': np.float64, 'af0': np.float64,
                   'health': np.int8, 'accuracy': np.int8}
    ALM_columns = {'week': np.int16, 'Toa': np.int32, 'e': np.float32, 'delta_i': np.float32, 'Wdot': np.float32,
                   'sqrtA': np.float32, 'W0': np.float32, 'w': np.float32, 'M0': np.float32, 'af0': np.float32,
                   'af1': np.float32, 'health': np.int8, 'Data_ID': np.int8}
    # TODO: delete error
    solves_columns = {'X': np.float64, 'Y': np.float64, 'Z': np.float64, 'cdt': np.float64, 'dt': np.float64,
                      'fval': np.float64, 'success': np.bool_, 'error': np.float64, 'calc_time': np.float32}

    week: int = None
    TOW: int = None
    iTOW: int = None
    last_iTOW: int = None
    stamp: TimeStamp = None

    other_data = dict()

    leapS = 18

    def __init__(self):
        self.counter = 0
        #TODO: задать индекс как (svId + 100 * gnssId.value) и использовать df1.update(df2)

        # pd.DataFrame(np.empty(0, dtype=np.dtype(list(X.items()))))
        stamp_columns = {'svId': np.int8, 'gnssId': object}
        init_sats_lines = [{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)]
                        # + [{'svId': j, 'gnssId': GNSS.GLONASS} for j in range(1, 25)],
        init_sats_df = pd.DataFrame(init_sats_lines).astype(stamp_columns)
        init_sats_df.set_index(get_df_indexes(init_sats_df), inplace=True)

        # TODO delete real_rho & Dt
        param_columns = stamp_columns | {'receiving_stamp': object, 'exist': np.bool_, 'is_old': np.bool_}
        self.ephemeris_parameters = create_table(param_columns | self.EPH_columns, init_sats_df)
        self.almanac_parameters = create_table(param_columns | self.ALM_columns, init_sats_df)
        general_nav_cols = stamp_columns | {name: object for name in ['receiving_stamp', 'NAV_ORB_stamp',
                                                                      'NAV_SAT_stamp', 'RXM_RAWX_stamp',
                                                                      'RXM_SVSI_stamp', 'RXM_MEASX_stamp']}
        self.navigation_parameters = create_table(param_columns | general_nav_cols | self.NAV_ORB_columns |
                                                  self.NAV_SAT_columns | self.RXM_RAWX_columns | self.RXM_SVSI_columns |
                                                  self.RXM_MEASX_columns, init_sats_df)
        data_columns = stamp_columns | {'xyz_stamp': object, 'X': np.float64, 'Y': np.float64, 'Z': np.float64,
                                        'lat': np.float64, 'lon': np.float64, 'alt': np.float64, 'pr_stamp': object,
                                        'pseuRangeRMSErr': np.int8, 'prMes': np.float64, 'prRes': np.float32,
                                        'real_rho': np.float64, 'Dt': np.float64, 'coord_score': np.float16,
                                        'nav_score': np.float16}
        self.ephemeris_data = create_table(data_columns, init_sats_df)
        self.almanac_data = create_table(data_columns, init_sats_df)
        full_solves_columns = {'week': int, 'TOW': int, 'sat_count': int} | \
                               {f'{"LM"}_{name}': type for name, type in self.solves_columns.items()} | \
                               {f'{"SQP"}_{name}': type for name, type in self.solves_columns.items()}
        self.ephemeris_solves = create_table(full_solves_columns, init_sats_df)
        self.almanac_solves = create_table(full_solves_columns, init_sats_df)

        # type_stamp = object  # object
        # for name in ['NAV_ORB_stamp', 'NAV_SAT_stamp', 'RXM_RAWX_stamp', 'RXM_SVSI_stamp', 'RXM_MEASX_stamp',
        #              'receiving_stamp']:
        #     self.navigation_parameters[name].astype(type_stamp)
        # self.almanac_parameters['receiving_stamp'].astype(type_stamp)
        # self.ephemeris_parameters['receiving_stamp'].astype(type_stamp)
        # for name in ['xyz_stamp', 'pr_stamp']:
        #     self.ephemeris_data[name].astype(type_stamp)
        #     self.almanac_data[name].astype(type_stamp)
        # ## Поменять типа ячеек времени на datetime
        # for name in ['NAV_ORB_stamp', 'NAV_SAT_TOW', 'RXM_RAWX_TOW', 'RXM_SVSI_TOW', 'RXM_MEASX_TOW', 'receiving_TOW']:
        #     self.navigation_data[name] = pd.to_datetime(self.navigation_data[name])
        # self.almanac_parameters['receiving_TOW'] = pd.to_datetime(self.almanac_parameters['receiving_TOW'])
        # self.ephemeris_parameters['receiving_TOW'] = pd.to_datetime(self.ephemeris_parameters['receiving_TOW'])

        # # TODO: integrate and delete
        # self.SFRBX_GPS_alm = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)],
        #                                   columns=['svId', 'gnssId', 'Data_ID', 'SV_ID', 'e', 'Toa', 'delta_i', 'Wdot',
        #                                            'health', 'sqrtA', 'W0', 'w', 'M0', 'af1', 'af0'])
        # self.SFRBX_GPS_eph = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)],
        #                                   columns=['svId', 'gnssId', 'week', 'accuracy', 'health', 'IODC', 'Tgd', 'Toc',
        #                                            'af2', 'af1', 'af0', 'IODE1', 'Crs', 'dn', 'M0', 'Cuc', 'e', 'Cus',
        #                                            'sqrtA', 'Toe', 'Cic', 'W0', 'Cis', 'i0', 'Crc', 'w', 'Wdot',
        #                                            'IODE2', 'IDOT'])
        # self.SFRBX_GPS_data = dict()
        # self.SFRBX_GLONASS_alm = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GLONASS} for j in range(1, 25)],
        #                                       columns=['svId', 'gnssId', 'n', 'lambda_n', 'delta_i_n', 'eps_n', 'M_n',
        #                                                'tau_n', 'Cn', 'Hn', 't_lambda_n', 'delta_T_n', 'delta_T_dot_n',
        #                                                'omega_n', 'ln'])
        # self.SFRBX_GLONASS_eph = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GLONASS} for j in range(1, 25)],
        #                                       columns=['svId', 'gnssId', 'B1', 'B2', 'KP', 'tau_c', 'tau_GPS', 'N4',
        #                                                'N', 'tk', 'x', 'dx', 'ddx', 'P1', 'tb', 'y', 'dy', 'ddy', 'Bn',
        #                                                'P2', 'gamma', 'z', 'dz', 'ddz', 'P', 'P3', 'ln', 'M', 'tau',
        #                                                'N_T', 'n', 'F_T', 'E', 'P4', 'dTau'])
        # self.SFRBX_GLONASS_data = dict()
        self.SFRBX_GPS_alm = pd.read_csv('sfrbx_gps_alm.csv', index_col=0)
        self.SFRBX_GPS_eph = pd.read_csv('sfrbx_gps_eph.csv', index_col=0)
        self.SFRBX_GLONASS_alm = pd.read_csv('sfrbx_glonass_alm.csv', index_col=0)
        self.SFRBX_GLONASS_eph = pd.read_csv('sfrbx_glonass_eph.csv', index_col=0)
        with open('sfrbx_gps_data.txt', 'r') as file:
            self.SFRBX_GPS_data = eval(file.read())

    def get_TOW(self):
        return self.TOW

    ### Обновление данных при приходе нового сообщения ###

    def update(self, message):
        self.counter += 1
        try:
            if not message:
                return
            if isinstance(message, pyubx2.ubxmessage.UBXMessage):
                return
            if isinstance(message, UBXMessages.UbxMessage):
                self.update_UBX(message)
            self.ADD_1_TABLES()
            if self.iTOW:
                self.TOW = self.iTOW // 1000
            if self.iTOW != self.last_iTOW:
                self.last_iTOW = self.iTOW
                return True
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            a=0
        return False

    def update_param(self, data, *attrs):
        for attr in attrs:
            if attr in data.keys() and hasattr(self, attr):
                setattr(self, attr, data[attr])

    def update_UBX(self, message):
        self.stamp: TimeStamp = message.receiving_stamp
        if isinstance(message, UBXMessages.AID_ALM | UBXMessages.AID_EPH):
            table = self.almanac_parameters if isinstance(message, UBXMessages.AID_ALM) else self.ephemeris_parameters
            if message.data:
                assert isinstance(message.data, dict)
                message.data.update({'is_old': 0, 'exist': 1})
                update_table_line(table, message.data, *message.stamp, message.receiving_stamp)
            else:
                update_table_line(table, {'is_old': 1}, *message.stamp, None)
        elif isinstance(message,
                        UBXMessages.NAV_SAT | UBXMessages.NAV_ORB | UBXMessages.RXM_RAWX | UBXMessages.RXM_SVSI |
                        UBXMessages.RXM_MEASX):
            assert isinstance(message.data, dict)
            self.update_param(message.data, 'iTOW', 'week')
            if message.satellites:
                for stamp, line in message.satellites.items():
                    update_table_line(self.navigation_parameters, line, *stamp, message.receiving_stamp,
                                      message.__class__.__name__ + '_stamp')
            self.navigation_parameters['receiving_stamp'] = self.navigation_parameters.apply(
                lambda row: custom_min(row['NAV_ORB_stamp'], row['NAV_SAT_stamp'], row['RXM_RAWX_stamp'],
                                       row['RXM_SVSI_stamp']), axis=1)
            if isinstance(message, UBXMessages.RXM_RAWX):
                self.calc_navigation_task()
        elif isinstance(message, UBXMessages.NAV_TIMEGPS | UBXMessages.NAV_POSECEF | UBXMessages.NAV_VELECEF):
            assert isinstance(message.data, dict)
            self.update_param(message.data, 'iTOW', 'week', 'leapS')
            if 'leapS' in message.data.keys():
                Constants.leapS = message.data['leapS']
            self.other_data[message.__class__.__name__.lower()] = message.data
        elif isinstance(message, UBXMessages.RXM_SFRBX):
            assert isinstance(message.data, dict)
            try: # TODO: править очень жестко
                if message.data['gnssId'] == GNSS.GPS:
                    if 'name' in message.signal.keys():
                        self.SFRBX_GPS_data.update(message.signal)
                        # TODO: add configs and health and ion
                    elif message.data['id'] == 0:
                        update_table_line(self.SFRBX_GPS_eph, message.signal,
                                          message.data['svId'], GNSS.GPS, None)
                    else:
                        update_table_line(self.SFRBX_GPS_alm, message.signal,
                                          message.signal['SV_ID'], GNSS.GPS, None)
                elif message.data['gnssId'] == GNSS.GLONASS:
                    if message.data['id'] == 0:
                        update_table_line(self.SFRBX_GLONASS_eph, message.signal,
                                          message.data['svId'], GNSS.GLONASS, None)
                    else:
                        update_table_line(self.SFRBX_GLONASS_alm, message.signal,
                                          message.data['id'], GNSS.GLONASS, None)
            except Exception as e:
                print(e)
                print(traceback.format_exc())
                a = 0
        match self.counter % 100:
            case 1:
                self.SFRBX_GPS_eph.to_csv('sfrbx_gps_eph.csv')
            case 2:
                self.SFRBX_GPS_alm.to_csv('sfrbx_gps_alm.csv')
            case 3:
                self.SFRBX_GLONASS_eph.to_csv('sfrbx_glonass_eph.csv')
            case 4:
                self.SFRBX_GLONASS_alm.to_csv('sfrbx_glonass_alm.csv')
            case 5:
                with open('sfrbx_gps_data.txt', 'w') as file:
                    file.write(str(self.SFRBX_GPS_data))


    def calc_navigation_task(self):

        nav_cols = ['svId', 'gnssId', 'pr_stamp', 'pseuRangeRMSErr', 'prMes', 'prRes', 'nav_score', 'alm_score',
                    'eph_score']
        coord_cols = ['svId', 'gnssId', 'xyz_stamp', 'X', 'Y', 'Z', 'lat', 'lon', 'alt', 'real_rho', 'Dt']
        nav_data = pd.DataFrame(self.navigation_parameters.apply(calc_nav, axis=1).to_list(), columns=nav_cols)
        eph_coord_data = pd.DataFrame(
            self.ephemeris_parameters.apply(lambda row: calc_coords(row, self.stamp, SCC.calc_gps_eph),
                                            axis=1).to_list(),
            columns=coord_cols)
        alm_coord_data = pd.DataFrame(
            self.almanac_parameters.apply(lambda row: calc_coords(row, self.stamp, SCC.calc_gps_alm), axis=1).to_list(),
            columns=coord_cols)
        self.ephemeris_data = pd.merge(eph_coord_data, nav_data, on=['svId', 'gnssId']). \
            drop(columns=['alm_score']).rename(columns={'eph_score': 'coord_score'})
        self.almanac_data = pd.merge(alm_coord_data, nav_data, on=['svId', 'gnssId']). \
            drop(columns=['eph_score']).rename(columns={'alm_score': 'coord_score'})
        calc_receiver_coordinates(self.ephemeris_data, self.ephemeris_solves, self.stamp)
        calc_receiver_coordinates(self.almanac_data, self.almanac_solves, self.stamp)

    def ADD_1_TABLES(self):
        self.navigation_parameters1 = self.navigation_parameters[['svId', 'gnssId', 'receiving_stamp', 'health', 'cno',
                                                                  'ephUsability', 'ephSource', 'almUsability',
                                                                  'almSource', 'prRes', 'qualityInd', 'svUsed', 'prMes',
                                                                  'ura', 'almAge', 'ephAge', 'orbitSource', 'ephAvail',
                                                                  'almAvail', 'mpathIndic', 'pseuRangeRMSErr',
                                                                  'locktime', 'codePhase', 'wholeChips', 'fracChips']]
        # self.navigation_parameters1.rename(columns={'pseuRangeRMSErr': 'RMSErrInd'}, inplace=True)
        self.navigation_parameters1.loc[:, 'codePhase'] = (
            (self.navigation_parameters['intCodePhase'] + self.navigation_parameters['codePhase']).round(5))
        self.navigation_parameters1.loc[:, 'prMes'] = self.navigation_parameters1['prMes'].round(4)
        # self.navigation_parameters1[['prRes']] = self.navigation_parameters1[['prRes']].round(3)
        self.navigation_parameters1.loc[:, 'prRes'] = self.navigation_parameters1['prRes'].round(3)

        self.ephemeris_parameters1 = self.ephemeris_parameters[['svId', 'gnssId', 'receiving_stamp', 'exist',
                                                                'is_old', 'week', 'Toe', 'Toc', 'Wdot',
                                                                'dn', 'i0', 'e', 'sqrtA', 'w', 'Tgd',
                                                                'health', 'accuracy']]
        self.almanac_parameters1 = self.almanac_parameters[['svId', 'gnssId', 'receiving_stamp', 'exist', 'is_old',
                                                            'week', 'Toa', 'e', 'delta_i', 'Wdot', 'sqrtA',
                                                            'w', 'M0', 'af0', 'af1', 'health', 'Data_ID']]

        self.ephemeris_data1 = self.ephemeris_data.copy()
        self.ephemeris_data1[['lat', 'lon', 'alt', 'prRes', 'nav_score', 'coord_score']] = \
            self.ephemeris_data1[['lat', 'lon', 'alt', 'prRes', 'nav_score', 'coord_score']].round(2)
        self.ephemeris_data1['RMSpr'] = (
            self.ephemeris_data1.apply(lambda row: calc_pseu_RMS_error(row['pseuRangeRMSErr']), axis=1))
        self.almanac_data1 = self.almanac_data.copy()
        self.almanac_data1[['lat', 'lon', 'alt', 'prRes', 'nav_score', 'coord_score']] = \
            self.almanac_data1[['lat', 'lon', 'alt', 'prRes', 'nav_score', 'coord_score']].round(2)
        self.almanac_data1['RMSpr'] = (
            self.almanac_data1.apply(lambda row: calc_pseu_RMS_error(row['pseuRangeRMSErr']), axis=1))

        self.ephemeris_solves1 = self.ephemeris_solves.copy()
        self.ephemeris_solves1.drop(columns=['LM_cdt', 'SQP_cdt'], inplace=True)
        self.ephemeris_solves1[['LM_fval', 'SQP_fval']] = \
            self.ephemeris_solves1[['LM_fval', 'SQP_fval']].apply(
                lambda col: col.map(lambda x: f"{x:.4e}"))  # .applymap(lambda x: f"{x:.4e}")
        cols = [f'{type}_{name}' for type in ['LM', 'SQP'] for name in ['X', 'Y', 'Z']]
        self.ephemeris_solves1[cols] = self.ephemeris_solves1[cols].round(1)

        self.almanac_solves1 = self.almanac_solves.copy()
        self.almanac_solves1.drop(columns=['LM_cdt', 'SQP_cdt'], inplace=True)
        self.almanac_solves1[['LM_fval', 'SQP_fval']] = \
            self.almanac_solves1[['LM_fval', 'SQP_fval']].apply(
                lambda col: col.map(lambda x: f"{x:.4e}"))  # .applymap(lambda x: f"{x:.4e}")
        self.almanac_solves1[cols] = self.almanac_solves1[cols].round(1)


## Для calc_navigation_task
def calc_nav_score(nav_row):
    quality = 2 if nav_row.qualityInd > 4 else (1 if nav_row.qualityInd == 4 else 0)
    score = (nav_row.health == 1) * (nav_row.visibility >= 2) * quality * (nav_row.prValid == True)
    if score:
        score *= nav_row.cno / (0.1 * abs(nav_row.prRes) + 1) / (nav_row.ura + 1) * (
                10 / nav_row.pseuRangeRMSErr)
    return score


def calc_eph_score(nav_row):
    return 100 / (5 + nav_row.ephAge) if (nav_row.ephSource == 1) * (nav_row.ephVal == True) else 0


def calc_alm_score(nav_row):
    return 100 / (5 + nav_row.almAge) if (nav_row.almSource == 1) * (nav_row.almVal == True) else 0


def calc_coords(param_row, stamp: TimeStamp, coord_func):
    # if param_row[['Wdot', 'e', 'sqrtA', 'M0', 'W0', 'w']].isna().any():
    if param_row.isna().any():
        xyz = None
    else:
        xyz = coord_func(param_row, stamp.TOW, stamp.week)
    if xyz:
        lla = Transformations.ecef2lla(*xyz)
    else:
        xyz = (np.nan, np.nan, np.nan)
        lla = (np.nan, np.nan, np.nan)
    rho = np.linalg.norm(np.array(xyz) - np.array(Constants.ECEF))
    return param_row.svId, param_row.gnssId, stamp, *xyz, *lla, rho, rho / Constants.c


def calc_nav(nav_row):
    return (nav_row.svId, nav_row.gnssId, nav_row.receiving_stamp, nav_row.pseuRangeRMSErr, nav_row.prMes,
            nav_row.prRes, calc_nav_score(nav_row), calc_alm_score(nav_row), calc_eph_score(nav_row))
