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


def update_table_line(table, data, svId, gnssId, receiving_TOW, TOW_type='receiving_TOW'):
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
    if receiving_TOW:
        table.at[ind, TOW_type] = receiving_TOW
    for key, value in data.items():
        if key in table.columns:
            table.at[ind, key] = value
    a = 0


def custom_min(*args):
    try:
        arr = [x for x in args if x is not None and pd.notna(x)]
        if arr:
            return min(arr)
    except Exception as e:
        print(e)
    return np.nan


def calc_nav_score(nav):
    quality = 2 if nav.qualityInd > 4 else (1 if nav.qualityInd == 4 else 0)
    score = (nav.health == 1) * (nav.visibility >= 2) * quality * (nav.prValid == True)
    if score:
        score *= nav.cno / (0.1 * abs(nav.prRes) + 1) / (nav.ura + 1) * (10 / nav.pseuRangeRMSErr)
    return score


def calc_eph_score(nav):
    # TODO: usability_for_time (here and alm_func)

    # using_time = (datetime.now() - nav.NAV_ORB_TOW).total_seconds() / 60 # for EPH table
    # usability_for_time = ((nav.almUsability - 1) * 15 > using_time) # но если unknown?
    score = (nav.ephSource == 1) * (nav.ephVal == True)
    if score:
        score *= 100 / (5 + nav.ephAge)
    return {'eph_score': score}, nav.svId, nav.gnssId,


def calc_alm_score(nav):
    # using_time = (datetime.now() - nav.NAV_ORB_TOW).total_seconds() / 60 # for EPH table
    # usability_for_time = ((nav.ephUsability - 1) * 15 > using_time) # но если unknown?
    score = (nav.almSource == 1) * (nav.almVal == True)
    if score:
        score *= 100 / (5 + nav.almAge)
    return {'alm_score': score}, nav.svId, nav.gnssId,


class Storage:
    #TODO отслеживать все сообщения или вычислять все после какого-то одного, или в определенный момент?
    cycle_messages = {UBXMessages.NAV_SAT: False, UBXMessages.NAV_ORB: False, UBXMessages.RXM_RAWX: False,
                      UBXMessages.RXM_SVSI: False}

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

    week: int = None
    TOW: int = None
    iTOW: int = None
    last_iTOW: int = None

    other_data = dict()

    this_stamp_messages = []
    clear_messages = False

    def __init__(self):
        self.eph_res = ['', '']
        self.alm_res = ['', '']
        init_gps_lines = [{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)]

        # TODO delete real_rho & Dt
        self.ephemeris_data = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId', 'receiving_TOW', 'TOW', 'coordinates', 'lla', 'real_rho', 'Dt', 'exist', 'is_old',
                     'eph_score'] + self.EPH_columns
        )
        self.almanac_data = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId', 'receiving_TOW', 'TOW', 'coordinates', 'lla', 'real_rho', 'Dt', 'exist', 'is_old',
                     'alm_score'] + self.ALM_columns
        )
        self.navigation_data = pd.DataFrame(
            init_gps_lines,
            columns=pd.Series(
                # General: receiving_TOW = max(all TOW) for every cycle
                ['svId', 'gnssId', 'nav_score',
                 'NAV_ORB_TOW', 'NAV_SAT_TOW', 'RXM_RAWX_TOW', 'RXM_SVSI_TOW', 'RXM_MEASX_TOW', 'receiving_TOW'] +
                self.NAV_ORB_columns + self.NAV_SAT_columns + self.RXM_RAWX_columns + self.RXM_SVSI_columns +
                self.RXM_MEASX_columns
            ).drop_duplicates().to_list()
        )
        ## Поменять типа ячеек времени на datetime
        for name in ['NAV_ORB_TOW', 'NAV_SAT_TOW', 'RXM_RAWX_TOW', 'RXM_SVSI_TOW', 'RXM_MEASX_TOW', 'receiving_TOW']:
            self.navigation_data[name] = pd.to_datetime(self.navigation_data[name])
        self.almanac_data['receiving_TOW'] = pd.to_datetime(self.almanac_data['receiving_TOW'])
        self.ephemeris_data['receiving_TOW'] = pd.to_datetime(self.ephemeris_data['receiving_TOW'])

    def get_TOW(self):
        return self.TOW

    def update_satellites_score(self):
        self.navigation_data['nav_score'] = self.navigation_data.apply(calc_nav_score, axis=1)
        for line in self.navigation_data.apply(calc_eph_score, axis=1):
            update_table_line(self.ephemeris_data, *line, receiving_TOW=None)
        for line in self.navigation_data.apply(calc_alm_score, axis=1):
            update_table_line(self.almanac_data, *line, receiving_TOW=None)

    def calc_satellites_coordinates(self):
        self.calc_sats_one_system(self.almanac_data, SCC.calc_sat_alm)
        self.calc_sats_one_system(self.ephemeris_data, SCC.calc_sat_eph)
        # self.almanac_data['coordinates'] = \
        #     self.almanac_data.apply(lambda row: SCC.calc_sat_alm(row, self.TOW, self.week) if row.exist else None,
        #                             axis=1)
        # self.almanac_data['lla'] = self.almanac_data.apply(lambda row: Transformations.ecef2lla(*row.coordinates),
        #                                                    axis=1)
        # self.almanac_data['real_rho'] = \
        #     self.almanac_data.apply(lambda row: np.linalg.norm(np.array(row.coordinates) - np.array(Constants.ECEF)),
        #                             axis=1)
        # self.almanac_data['Dt'] = (self.almanac_data['real_rho'] - self.navigation_data['prMes']) / Constants.c
        # self.almanac_data['TOW'] = self.TOW
        #
        # self.ephemeris_data['coordinates'] = \
        #     self.ephemeris_data.apply(lambda row: SCC.calc_sat_eph(row, self.TOW, self.week) if row.exist else None,
        #                               axis=1)
        # self.ephemeris_data['lla'] = self.ephemeris_data.apply(lambda row: Transformations.ecef2lla(*row.coordinates),
        #                                                    axis=1)
        # self.ephemeris_data['real_rho'] = \
        #     self.ephemeris_data.apply(lambda row: np.linalg.norm(np.array(row.coordinates) - np.array(Constants.ECEF)),
        #                               axis=1)
        # self.ephemeris_data['Dt'] = (self.ephemeris_data['real_rho'] - self.navigation_data['prMes']) / Constants.c
        # self.ephemeris_data['TOW'] = self.TOW

    def calc_sats_one_system(self, table, calc_func):
        table['coordinates'] = \
            table.apply(lambda row: calc_func(row, self.TOW, self.week) if row.exist else None,
                                    axis=1)
        table['lla'] = table.apply(lambda row: Transformations.ecef2lla(*row.coordinates),
                                                           axis=1)
        table['real_rho'] = \
            table.apply(lambda row: np.linalg.norm(np.array(row.coordinates) - np.array(Constants.ECEF)),
                                    axis=1)
        table['Dt'] = (table['real_rho'] - self.navigation_data['prMes']) / Constants.c
        table['TOW'] = self.TOW

    def calc_receiver_coordinates(self, eph_flag=True):
        if eph_flag:
            coords = self.ephemeris_data[['svId', 'gnssId', 'coordinates', 'exist', 'eph_score']]
            coords = coords.rename(columns={'eph_score': 'coord_score'})
        else:
            coords = self.almanac_data[['svId', 'gnssId', 'coordinates', 'exist', 'alm_score']]
            coords = coords.rename(columns={'alm_score': 'coord_score'})

        coords = coords[
            (coords.exist == True) & coords['coordinates'].apply(lambda coords: not any(map(pd.isna, coords)))]

        navs = self.navigation_data[['svId', 'gnssId', 'prMes', 'nav_score']]
        navs = navs[navs.nav_score > 0]

        sats = pd.merge(coords[['svId', 'gnssId', 'coordinates', 'coord_score']],
                        navs[['svId', 'gnssId', 'prMes', 'nav_score']],
                        on=['svId', 'gnssId'])
        sats['score'] = sats.coord_score + sats.nav_score
        sats = sats.sort_values(by='score', ascending=False).head(Settings.MinimizingSatellitesCount)
        if len(sats) < Settings.MinimumMinimizingSatellitesCount:
            # TODO: return None
            return [f'sats count: {len(sats)}', f'sats count: {len(sats)}']
        data = [(x, y, z, P) for (x, y, z), P in zip(sats['coordinates'], sats['prMes'])]

        res1 = Minimizing.solve_navigation_task_LevMar(data)
        res1.ERROR = np.linalg.norm(res1.x[:-1] - Constants.ECEF)
        res1.dt = res1.x[-1] / Constants.c
        res1.lla = Transformations.ecef2lla(*res1.x[:-1])
        res2 = Minimizing.solve_navigation_task_SLSQP(data)
        res2.ERROR = np.linalg.norm(res2.x[:-1] - Constants.ECEF)
        res2.dt = res2.x[-1] / Constants.c
        res2.lla = Transformations.ecef2lla(*res2.x[:-1])
        # self.res3 = Minimizing.solve_navigation_task_GO(data) # ОЧЕНЬ ДОЛГО - 4 секунды
        # self.res3.ERROR = np.linalg.norm(self.res3.x[:-1] - Constants.ECEF)
        return res1, res2
        pass

    def calc_coordinates(self):
        if not self.TOW:
            return
        self.calc_satellites_coordinates()
        self.eph_res = self.calc_receiver_coordinates(eph_flag=True)
        self.alm_res = self.calc_receiver_coordinates(eph_flag=False)

    def update(self, message):
        ## для удобного просмотра #TODO: delete
        NAV = self.navigation_data[['svId', 'nav_score', 'receiving_TOW', 'health', 'visibility', 'ephUsability', 'cno',
                                    'prRes', 'qualityInd', 'svUsed', 'orbitSource', 'prMes', 'prValid', 'ura', 'ephVal',
                                    'almAge', 'ephAge', 'mpathIndic', 'dopplerMS', 'pseuRangeRMSErr']]
        EPH = self.ephemeris_data[['svId', 'eph_score', 'receiving_TOW', 'TOW', 'coordinates', 'lla', 'real_rho', 'Dt',
                                   'exist', 'is_old', 'health', 'accuracy']]
        ALM = self.almanac_data[['svId', 'alm_score', 'receiving_TOW', 'TOW', 'coordinates', 'lla', 'real_rho', 'Dt',
                                 'exist', 'is_old', 'health']]
        res = {
            'alm': {
                'LM': self.alm_res[0],
                'SQP': self.alm_res[1],
            },
            'eph': {
                'LM': self.eph_res[0],
                'SQP': self.eph_res[1],
            }
        }

        if not message:
            return
        if self.clear_messages:
            self.this_stamp_messages.clear()
            self.clear_messages = False
        self.this_stamp_messages.append(message)
        if isinstance(message, UBXMessages.UbxMessage):
            self.update_UBX(message)
        if self.iTOW:
            self.TOW = self.iTOW // 1000
        if all(self.cycle_messages.values()):
            for key in self.cycle_messages.keys():
                self.cycle_messages[key] = False
            self.calc_coordinates()
            self.clear_messages = True
            return self.this_stamp_messages
        return False

        # if self.iTOW != self.last_iTOW:  # отслеживание новой метки времени
        #     self.last_iTOW = self.iTOW

    def update_param(self, data, *attrs):
        for attr in attrs:
            if attr in data.keys():
                setattr(self, attr, data[attr])

    def update_UBX(self, message):
        if isinstance(message, UBXMessages.AID_ALM | UBXMessages.AID_EPH):
            table = self.almanac_data if isinstance(message, UBXMessages.AID_ALM) else self.ephemeris_data
            if message.data:
                assert isinstance(message.data, dict)
                message.data.update({'is_old': 0, 'exist': 1})
                update_table_line(table, message.data, *message.stamp, message.receiving_TOW)
            else:
                update_table_line(table, {'is_old': 1}, *message.stamp, None)
        elif isinstance(message,
                        UBXMessages.NAV_SAT | UBXMessages.NAV_ORB | UBXMessages.RXM_RAWX | UBXMessages.RXM_SVSI |
                        UBXMessages.RXM_MEASX):
            assert isinstance(message.data, dict)
            self.cycle_messages[type(message)] = True
            self.update_param(message.data, 'iTOW', 'week')
            if message.satellites:
                for stamp, line in message.satellites.items():
                    update_table_line(self.navigation_data, line, *stamp, message.receiving_TOW,
                                      message.__class__.__name__ + '_TOW')
            self.update_satellites_score()
            # TODO: время в datetime, а нужно в TOW
            self.navigation_data['receiving_TOW'] = self.navigation_data.apply(
                lambda row: custom_min(row['NAV_ORB_TOW'], row['NAV_SAT_TOW'], row['RXM_RAWX_TOW'],
                                       row['RXM_SVSI_TOW']), axis=1)

        elif isinstance(message, UBXMessages.NAV_TIMEGPS | UBXMessages.NAV_POSECEF | UBXMessages.NAV_VELECEF):
            assert isinstance(message.data, dict)
            self.update_param(message.data, 'iTOW', 'week')
            self.other_data[message.__class__.__name__.lower()] = message.data

        a = 0
