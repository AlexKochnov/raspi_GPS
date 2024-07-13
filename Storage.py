from enum import Enum
import numpy as np
import pandas as pd
from datetime import datetime, timedelta

# from Messages import *
# import Messages
import UBXMessages
from UtilsMessages import GNSS


def update_table_line(table, data, svId, gnssId, receiving_TOW, TOW_type='receiving_TOW'):
    ind = table.index[(table.svId == svId) & (table.gnssId == gnssId)]
    if not len(ind):
        data['svId'] = svId
        data['gnssId'] = gnssId
        #TODO добавить другие ГНСС системы

        # table.loc[len(table)] = data
        # table = pd.concat([table, pd.DataFrame([data])], ignore_index=True)
        return
    ind = ind[0]
    if receiving_TOW:
        table.at[ind, TOW_type] = receiving_TOW
    for key, value in data.items():
        if key in table.columns:
            table.at[ind, key] = value


def custom_max(*args):
    try:
        arr = [x for x in args if x is not None and pd.notna(x)]
        if arr:
            return max(arr)
    except Exception as e:
        print(e)
    return np.nan

def calc_nav_score(row):
    score = (row.health == 1) * (row.visibility >= 2) *1
    return 0

def calc_eph_score(row):
    usability_time = (datetime.now() - row.receiving_TOW).total_seconds() / 60
    score =1

class Storage:
    ephemeris_data: pd.DataFrame
    almanac_data: pd.DataFrame
    nav_orb_data: pd.DataFrame
    nav_sat_data: pd.DataFrame
    rxm_rawx_data: pd.DataFrame
    rxm_svsi_data: pd.DataFrame

    NAV_ORB_columns = ['health', 'visibility', 'ephUsability', 'ephSource', 'almUsability', 'almSource',
                       'anoAopUsability', 'type']
    NAV_SAT_columns = ['cno', 'elev', 'azim', 'prRes', 'qualityInd', 'svUsed', 'health', 'diffCorr', 'smoothed',
                       'orbitSource', 'ephAvail', 'almAvail']
    RXM_RAWX_columns = ['prMes', 'cpMes', 'doMes', 'freqId', 'locktime', 'cno', 'prStedv', 'cpStedv', 'doStedv',
                        'prValid', 'cpValid', 'halfCyc', 'subHalfCyc']
    RXM_SVSI_columns = ['azim', 'elev', 'ura', 'healthy', 'ephVal', 'almVal', 'notAvail', 'almAge', 'ephAge']
    EPH_columns = ['week', 'Toe', 'Toc', 'IODE1', 'IODE2', 'IODC', 'IDOT', 'Wdot', 'Crs', 'Crc', 'Cus', 'Cuc', 'Cis',
                   'Cic', 'dn', 'i0', 'e', 'sqrtA', 'M0', 'W0', 'w', 'Tgd', 'af2', 'af1', 'af0', 'health', 'accuracy']
    ALM_columns = ['week', 'Toa', 'e', 'delta_i', 'Wdot', 'sqrtA', 'W0', 'w', 'M0', 'af0', 'af1', 'health', 'Data_ID']

    TOW: int = None
    iTOW: int = None

    other_data = dict()

    def __init__(self):
        init_gps_lines = [{'svId': j, 'gnssId': GNSS.GPS} for j in range(1, 33)]
        self.ephemeris_data = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId', 'receiving_TOW', 'TOW', 'coordinates', 'is_old', 'eph_score'] + self.EPH_columns
        )
        self.almanac_data = pd.DataFrame(
            init_gps_lines,
            columns=['svId', 'gnssId', 'receiving_TOW', 'TOW', 'coordinates', 'is_old', 'alm_score'] + self.ALM_columns
        )
        self.navigation_data = pd.DataFrame(
            init_gps_lines,
            columns=pd.Series(
                    # General: receiving_TOW = max(all TOW) for every cycle
                    ['svId', 'gnssId', 'nav_score',
                     'NAV_ORB_TOW', 'NAV_SAT_TOW', 'RXM_RAWX_TOW', 'RXM_SVSI_TOW', 'receiving_TOW'] +
                    self.NAV_ORB_columns + self.NAV_SAT_columns + self.RXM_RAWX_columns + self.RXM_SVSI_columns
                ).drop_duplicates().to_list()
        )
        ## Поменять типа ячеек времени на datetime
        for name in ['NAV_ORB_TOW', 'NAV_SAT_TOW', 'RXM_RAWX_TOW', 'RXM_SVSI_TOW', 'receiving_TOW']:
            self.navigation_data[name] = pd.to_datetime(self.navigation_data[name])
        self.almanac_data['receiving_TOW'] = pd.to_datetime(self.almanac_data['receiving_TOW'])
        self.ephemeris_data['receiving_TOW'] = pd.to_datetime(self.ephemeris_data['receiving_TOW'])

    def get_TOW(self):
        return self.TOW


    def update_satellites_score(self):
        self.navigation_data['nav_score'] = self.navigation_data.apply(calc_nav_score, axis=1)

    def update(self, message):
        if not message:
            return
        if isinstance(message, UBXMessages.UbxMessage):
            self.update_UBX(message)
        if self.iTOW:
            self.TOW = self.iTOW // 1000

    def update_UBX(self, message):
        if isinstance(message, UBXMessages.AID_ALM):
            if message.data:
                message.data['is_old'] = False
                update_table_line(self.almanac_data, message.data, *message.stamp, message.receiving_TOW)
            else:
                update_table_line(self.almanac_data, {'is_old': True}, *message.stamp, None)
        elif isinstance(message, UBXMessages.AID_EPH):
            if message.data:
                update_table_line(self.ephemeris_data, message.data, *message.stamp, message.receiving_TOW)
        elif isinstance(message, UBXMessages.NAV_SAT | UBXMessages.NAV_ORB | UBXMessages.RXM_RAWX | UBXMessages.RXM_SVSI):
            if 'iTOW' in message.data.keys():
                self.iTOW = message.data['iTOW']
            if message.satellites:
                for stamp, line in message.satellites.items():
                    update_table_line(self.navigation_data, line, *stamp, message.receiving_TOW,
                                      message.__class__.__name__ + '_TOW')
            # self.update_satellites_score()
            # TODO: время в datetime, а не TOW
            self.navigation_data['receiving_TOW'] = self.navigation_data.apply(
                lambda row: custom_max(row['NAV_ORB_TOW'], row['NAV_SAT_TOW'], row['RXM_RAWX_TOW'],
                                       row['RXM_SVSI_TOW']), axis=1)

        elif isinstance(message, UBXMessages.NAV_TIMEGPS | UBXMessages.NAV_POSECEF | UBXMessages.NAV_VELECEF):
            self.iTOW = message.data['iTOW']
            self.other_data[message.__class__.__name__.lower()] = message.data


