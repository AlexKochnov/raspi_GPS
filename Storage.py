import math
from enum import Enum
import numpy as np
import pandas as pd
from datetime import datetime, timedelta

import Constants
import Minimizing
import Settings
import Transformations
# from Messages import *
# import Messages
import UBXMessages
from UtilsMessages import GNSS
import SatellitesCoordinateCalculator as SCC


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
            table.at[ind, key] = value


def apply_func_get(table, func):
    result = {}
    for ind, row in table.iterrows():
        result[(row.svId, row.gnssId)] = func(row)
    return result


def update_table(table, datas):
    t = datetime.now(tz=Constants.tz_utc)
    for stamp, data in datas.items():
        for key, value in data.items():
            table.loc[table.index[(table.svId == stamp[0]) & (table.gnssId == stamp[1])], key] = value
    t2 = datetime.now(tz=Constants.tz_utc)
    print(f'\t\tupdate: {t2-t}')


def apply_func_set(table, func):
    update_table(table, apply_func_get(table, func))


def custom_min(*args):
    arr = [x for x in args if x is not None and pd.notna(x)]
    if arr:
        return min(arr)
    return np.nan


def calc_nav_score(nav_row):
    quality = 2 if nav_row.qualityInd > 4 else (1 if nav_row.qualityInd == 4 else 0)
    score = (nav_row.health == 1) * (nav_row.visibility >= 2) * quality * (nav_row.prValid == True)
    if score:
        score *= nav_row.cno / (0.1 * abs(nav_row.prRes) + 1) / (nav_row.ura + 1) * (10 / nav_row.pseuRangeRMSErr)
    return score


def calc_eph_score(nav_row):
    # TODO: usability_for_time (here and alm_func)

    # using_time = (datetime.now() - nav.NAV_ORB_TOW).total_seconds() / 60 # for EPH table
    # usability_for_time = ((nav.almUsability - 1) * 15 > using_time) # но если unknown?
    score = (nav_row.ephSource == 1) * (nav_row.ephVal == True)
    if score:
        score *= 100 / (5 + nav_row.ephAge)
    return score


def calc_alm_score(nav_row):
    # using_time = (datetime.now() - nav.NAV_ORB_TOW).total_seconds() / 60 # for EPH table
    # usability_for_time = ((nav.ephUsability - 1) * 15 > using_time) # но если unknown?
    score = (nav_row.almSource == 1) * (nav_row.almVal == True)
    if score:
        score *= 100 / (5 + nav_row.almAge)
    return score


def prepare_navigation_data(nav_row, param_row, stamp, coord_func, score_func):
    xyz = coord_func(param_row.iloc[0], stamp % Constants.week_seconds, stamp // Constants.week_seconds)
    if xyz:
        lla = Transformations.ecef2lla(*xyz)
    else:
        xyz = (np.nan, np.nan, np.nan)
        lla = (np.nan, np.nan, np.nan)
    rho = np.linalg.norm(np.array(xyz) - np.array(Constants.ECEF))
    result = {
        'xyz_stamp': stamp, 'X': xyz[0], 'Y': xyz[1], 'Z': xyz[2], 'lat': lla[0], 'lon': lla[1], 'alt': lla[2],
        'pr_stamp': nav_row['receiving_stamp'], 'pseuRangeRMSErr': nav_row['pseuRangeRMSErr'],
        'prMes': nav_row['prMes'], 'prRes': nav_row['prRes'],
        'real_rho': rho, 'Dt': rho / Constants.c,
        'coord_score': score_func(nav_row=nav_row.iloc[0]),
        'nav_score': calc_nav_score(nav_row=nav_row.iloc[0])
    }
    return result


def get_row(table, svId, gnssId):
    return table[(table.svId == svId) & (table.gnssId == gnssId)]

def calc_receiver_coordinates(data_table: pd.DataFrame, solve_table: pd.DataFrame, stamp):
    sats = data_table.copy()
    sats = sats[(sats.nav_score > 0) & (sats.coord_score > 0) &
                (stamp - sats.xyz_stamp < 10) & (stamp - sats.pr_stamp < 10)]
    sats['score'] = sats.coord_score + sats.nav_score
    sats = sats.sort_values(by='score', ascending=False).head(Settings.MinimizingSatellitesCount)
    data = [(row.X, row.Y, row.Z, row.prMes) for row in sats.iterrows()]
    SOLVE = {'week': stamp // Constants.week_seconds, 'TOW': stamp % Constants.week_seconds, 'sat_count': len(data)}
    if len(data) >= Settings.MinimumMinimizingSatellitesCount:
        for name, func in [('LM', Minimizing.solve_navigation_task_LevMar),
                           'SQP', Minimizing.solve_navigation_task_SLSQP]:
            res = func(data)
            solve = {
                'X': res.x[0], 'Y': res.x[1], 'Z': res.x[2], 'cdt': res.x[3], 'dt': res.x[3]/Constants.c,
                'fval': sum(res.fval) if isinstance(res.fval, list | tuple) else res.fval,
                'error': np.linalg.norm(np.array(res.x[:3]) - np.array(Constants.ECEF))
            }
            solve = {f'{key}_{name}' for key, value in solve.items()}
            SOLVE |= solve
    # else:
    #     SOLVE |= {f'{name}_{'LM'}': np.nan for name in Storage.solves_columns}
    #     SOLVE |= {f'{name}_{'SQP'}': np.nan for name in Storage.solves_columns}
    solve_table.loc[len(solve_table)] = SOLVE





class Storage:
    NAV_ORB_columns = ['health', 'visibility', 'ephUsability', 'ephSource', 'almUsability', 'almSource',
                       'anoAopUsability', 'type']
    NAV_SAT_columns = ['cno', 'elev', 'azim', 'prRes', 'qualityInd', 'svUsed', 'health', 'diffCorr', 'smoothed',
                       'orbitSource', 'ephAvail', 'almAvail']
    RXM_RAWX_columns = ['prMes', 'cpMes', 'doMes', 'freqId', 'locktime', 'cno', 'prStedv', 'cpStedv', 'doStedv',
                        'prValid', 'cpValid', 'halfCyc', 'subHalfCyc']
    RXM_SVSI_columns = ['azim', 'elev', 'ura', 'healthy', 'ephVal', 'almVal', 'notAvail', 'almAge', 'ephAge']
    RXM_MEASX_columns = ['cno', 'mpathIndic', 'dopplerMS', 'dopplerHz', 'wholeChips', 'fracChips', 'codePhase',
                         'intCodePhase', 'pseuRangeRMSErr']
    EPH_columns = ['week', 'Toe', 'Toc', 'IODE1', 'IODE2', 'IODC', 'IDOT', 'Wdot', 'Crs', 'Crc', 'Cus', 'Cuc', 'Cis',
                   'Cic', 'dn', 'i0', 'e', 'sqrtA', 'M0', 'W0', 'w', 'Tgd', 'af2', 'af1', 'af0', 'health', 'accuracy']
    ALM_columns = ['week', 'Toa', 'e', 'delta_i', 'Wdot', 'sqrtA', 'W0', 'w', 'M0', 'af0', 'af1', 'health', 'Data_ID']
    # TODO: delete error
    solves_columns = ['X', 'Y', 'Z', 'cdt', 'dt', 'fval', 'error']

    week: int = None
    TOW: int = None
    iTOW: int = None
    last_iTOW: int = None

    other_data = dict()

    leapS = 18

    def __init__(self):
        init_gps_lines = [{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)]

        # TODO delete real_rho & Dt
        self.ephemeris_parameters = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId', 'receiving_stamp', 'exist', 'is_old'] + self.EPH_columns
        )
        self.almanac_parameters = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId', 'receiving_stamp', 'exist', 'is_old'] + self.ALM_columns
        )
        self.navigation_parameters = pd.DataFrame(
            init_gps_lines,
            columns=pd.Series(
                # General: receiving_TOW = max(all TOW) for every cycle
                ['svId', 'gnssId', 'receiving_stamp',
                 'NAV_ORB_stamp', 'NAV_SAT_stamp', 'RXM_RAWX_stamp', 'RXM_SVSI_stamp', 'RXM_MEASX_stamp'] +
                self.NAV_ORB_columns + self.NAV_SAT_columns + self.RXM_RAWX_columns + self.RXM_SVSI_columns +
                self.RXM_MEASX_columns
            ).drop_duplicates().to_list()
        )
        self.ephemeris_data = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId',
                     'xyz_stamp', 'X', 'Y', 'Z', 'lat', 'lon', 'alt',
                     'pr_stamp', 'pseuRangeRMSErr', 'prMes', 'prRes',
                     'real_rho', 'Dt', 'coord_score', 'nav_score']
        )
        self.almanac_data = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId',
                     'xyz_stamp', 'X', 'Y', 'Z', 'lat', 'lon', 'alt',
                     'pr_stamp', 'pseuRangeRMSErr', 'prMes', 'prRes',
                     'real_rho', 'Dt', 'coord_score', 'nav_score']
        )
        self.ephemeris_solves = pd.DataFrame(
            columns=['week', 'TOW', 'sat_count'] + [f'{name}_{type}' for name in self.solves_columns for type in ['SQP', 'LM']]
        )
        self.almanac_solves = pd.DataFrame(
            columns=['week', 'TOW', 'sat_count'] + [f'{name}_{type}' for name in self.solves_columns for type in ['SQP', 'LM']]
        )

        # ## Поменять типа ячеек времени на datetime
        # for name in ['NAV_ORB_stamp', 'NAV_SAT_TOW', 'RXM_RAWX_TOW', 'RXM_SVSI_TOW', 'RXM_MEASX_TOW', 'receiving_TOW']:
        #     self.navigation_data[name] = pd.to_datetime(self.navigation_data[name])
        # self.almanac_parameters['receiving_TOW'] = pd.to_datetime(self.almanac_parameters['receiving_TOW'])
        # self.ephemeris_parameters['receiving_TOW'] = pd.to_datetime(self.ephemeris_parameters['receiving_TOW'])

    def get_TOW(self):
        return self.TOW

    ### Обновление данных при приходе нового сообщения ###

    def update(self, message):
        if not message:
            return
        if isinstance(message, UBXMessages.UbxMessage):
            self.update_UBX(message)
        if self.iTOW:
            self.TOW = self.iTOW // 1000
        return False

    def update_param(self, data, *attrs):
        for attr in attrs:
            if attr in data.keys() and hasattr(self, attr):
                setattr(self, attr, data[attr])

    def update_UBX(self, message):
        self.stamp = message.receiving_stamp
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

    def calc_navigation_task(self):
        t1 = datetime.now()
        apply_func_set(table=self.ephemeris_data,
                       func=lambda row: prepare_navigation_data(
                           nav_row=get_row(self.navigation_parameters, row.svId, row.gnssId),
                           param_row=get_row(self.ephemeris_parameters, row.svId, row.gnssId),
                           stamp=self.stamp, coord_func=SCC.calc_sat_eph, score_func=calc_eph_score
                       )
                       )
        t2 = datetime.now()
        calc_receiver_coordinates(self.ephemeris_data, self.ephemeris_solves, self.stamp)
        t3 = datetime.now()
        apply_func_set(table=self.almanac_data,
                       func=lambda row: prepare_navigation_data(
                           nav_row=get_row(self.navigation_parameters, row.svId, row.gnssId),
                           param_row=get_row(self.almanac_parameters, row.svId, row.gnssId),
                           stamp=self.stamp, coord_func=SCC.calc_sat_alm, score_func=calc_alm_score
                       )
                       )
        t4 = datetime.now()
        print(f'\t eph: {t2-t1}, solEL {t3-t2}, almanac: {t4-t3}')


