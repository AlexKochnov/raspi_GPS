import os
import pickle
import traceback
import numpy as np
import pandas as pd
from datetime import datetime

import pyubx2

import Constants
import Minimizing
import Settings
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
    return row.svId + GNSS(row.gnssId).value * 100


def create_table(columns: dict, init_sats_lines: pd.DataFrame = pd.DataFrame()):
    types = np.dtype(list(columns.items()))
    table = pd.DataFrame(np.empty(len(init_sats_lines), dtype=types))
    if not init_sats_lines.empty:
        table.set_index(init_sats_lines.index, inplace=True)
        table.update(init_sats_lines)
    return table


def create_index_table(data):
    table = pd.DataFrame(data)
    table.set_index(get_df_indexes(table), inplace=True)
    return table


def make_init_df(*gnss_s: GNSS):
    def init_lines_to_df(lines: list[dict]):
        init_df = pd.DataFrame(lines).astype(SCL.stamp_columns)
        init_df.set_index(get_df_indexes(init_df), inplace=True)
        return init_df

    lines = []
    for gnss in gnss_s:
        lines += [{'svId': j + 1, 'gnssId': gnss} for j in range(get_GNSS_len(gnss))]
    return init_lines_to_df(lines)


SFRBX_DATA_PATH = 'sfrbx_data.plk'


class Storage:
    week: int = None
    TOW: int = None
    iTOW: int = None
    last_iTOW: int = None
    time_stamp: TimeStamp = None

    other_data = dict()

    leapS = 18

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

        # TODO: integrate and delete
        if os.path.exists(SFRBX_DATA_PATH):
            with open(SFRBX_DATA_PATH, 'rb') as file:
                self.SFRBX_GPS_eph = pickle.load(file)
                self.SFRBX_GPS_alm = pickle.load(file)
                self.SFRBX_GLONASS_eph = pickle.load(file)
                self.SFRBX_GLONASS_alm = pickle.load(file)
                self.SFRBX_GPS_data = pickle.load(file)
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
            lambda row: custom_min(row['NAV_ORB_stamp'], row['NAV_SAT_stamp'], row['RXM_RAWX_stamp'],
                                   row['RXM_SVSI_stamp']), axis=1)

    def update_UBX(self, message):
        self.time_stamp: TimeStamp = message.receiving_stamp
        if isinstance(message, UBXMessages.AID_ALM | UBXMessages.AID_EPH):
            table = self.almanac_parameters if isinstance(message, UBXMessages.AID_ALM) else self.ephemeris_parameters
            data = message.stamp | {'receiving_stamp': message.receiving_stamp}
            if message.data:
                table.update(create_index_table([message.data | {'is_old': False, 'exist': True} | data]))
            else:
                table.update(create_index_table([{'is_old': True} | data]))
        elif isinstance(message, UBXMessages.NAV_SAT | UBXMessages.NAV_ORB | UBXMessages.RXM_RAWX |
                                 UBXMessages.RXM_SVSI | UBXMessages.RXM_MEASX):
            self.update_param(message.data, 'iTOW', 'week')
            if message.satellites:
                self.navigation_parameters.update(create_index_table(message.satellites))
                self.check_nav_time()
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
            try:  # TODO: править очень жестко
                if message.data['gnssId'] == GNSS.GPS:
                    if 'name' in message.signal.keys():
                        self.SFRBX_GPS_data.update(message.signal)
                        # TODO: add configs and health and ion processing
                    elif message.data['id'] == 0:
                        #TODO: update_line() -> df.update()
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
        if self.counter % 100 == 50:
            with open(SFRBX_DATA_PATH, 'wb') as file:
                pickle.dump(self.SFRBX_GPS_eph, file)
                pickle.dump(self.SFRBX_GPS_alm, file)
                pickle.dump(self.SFRBX_GLONASS_eph, file)
                pickle.dump(self.SFRBX_GLONASS_alm, file)
                pickle.dump(self.SFRBX_GPS_data, file)

    def calc_navigation_task(self):
        try:
            self.ephemeris_data, self.almanac_data = \
                make_ae_nav_data(self.navigation_parameters, self.ephemeris_parameters, self.almanac_parameters,
                                 self.time_stamp)
            calc_receiver_coordinates(self.ephemeris_data, self.ephemeris_solves, self.time_stamp)
            calc_receiver_coordinates(self.almanac_data, self.almanac_solves, self.time_stamp)
        except Exception as e:
            print(e)
            print(tr := traceback.format_exc())
            a = 0

    def ADD_GUI_TABLES(self):
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
                                              columns=['svId', 'gnssId', 'n', 'lambda_n', 'delta_i_n', 'eps_n', 'M_n',
                                                       'tau_n', 'Cn', 'Hn', 't_lambda_n', 'delta_T_n', 'delta_T_dot_n',
                                                       'omega_n', 'ln'])
        self.SFRBX_GLONASS_eph = pd.DataFrame([{'svId': j, 'gnssId': GNSS.GLONASS} for j in range(1, 25)],
                                              columns=['svId', 'gnssId', 'B1', 'B2', 'KP', 'tau_c', 'tau_GPS', 'N4',
                                                       'N', 'tk', 'x', 'dx', 'ddx', 'P1', 'tb', 'y', 'dy', 'ddy', 'Bn',
                                                       'P2', 'gamma', 'z', 'dz', 'ddz', 'P', 'P3', 'ln', 'M', 'tau',
                                                       'N_T', 'n', 'F_T', 'E', 'P4', 'dTau'])
        self.SFRBX_GLONASS_data = dict()
