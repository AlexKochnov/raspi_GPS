import numpy as np
import pandas as pd

import Constants
import SatellitesCoordinateCalculator as SCC
import Transformations
from TimeStamp import TimeStamp


""" 
Для Storage.calc_navigation_task 
"""


def make_ae_nav_data(navigation_parameters, ephemeris_parameters, almanac_parameters, time_stamp):
    nav_cols = ['svId', 'gnssId', 'pr_stamp', 'pseuRangeRMSErr', 'prMes', 'prRes', 'nav_score', 'alm_score',
                'eph_score']
    coord_cols = ['svId', 'gnssId', 'xyz_stamp', 'X', 'Y', 'Z', 'lat', 'lon', 'alt', 'real_rho', 'Dt']
    nav_data = pd.DataFrame(navigation_parameters.apply(calc_nav, axis=1).to_list(), columns=nav_cols)
    eph_coord_data = pd.DataFrame(
        ephemeris_parameters.apply(lambda row: calc_coords(row, time_stamp, SCC.calc_gps_eph), axis=1).to_list(),
        columns=coord_cols)
    alm_coord_data = pd.DataFrame(
        almanac_parameters.apply(lambda row: calc_coords(row, time_stamp, SCC.calc_gps_alm), axis=1).to_list(),
        columns=coord_cols)
    ephemeris_data = pd.merge(eph_coord_data, nav_data, on=['svId', 'gnssId']). \
        drop(columns=['alm_score']).rename(columns={'eph_score': 'coord_score'})
    almanac_data = pd.merge(alm_coord_data, nav_data, on=['svId', 'gnssId']). \
        drop(columns=['eph_score']).rename(columns={'alm_score': 'coord_score'})
    return ephemeris_data, almanac_data


def calc_nav_score(nav_row):
    quality = 2 if nav_row.qualityInd > 4 else (1 if nav_row.qualityInd == 4 else 0)
    score = (nav_row.health == 1) * (nav_row.visibility >= 2) * quality * (nav_row.prValid == True)
    if score:
        score *= 100*nav_row.cno / (abs(nav_row.prRes) + 1) * (nav_row.prRMSer) #TODO: RMS error (not index)
    return score


def calc_eph_score(nav_row):
    if (nav_row.ephSource == 1) * (nav_row.ephAvail == True):
        return nav_row.ephUsability if nav_row.ephUsability != 31 else 10
    else:
        return 0


def calc_alm_score(nav_row):
    if (nav_row.almSource == 1) * (nav_row.almAvail == True):
        return nav_row.almUsability if nav_row.almUsability != 31 else 10
    else:
        return 0


def calc_coords(param_row, stamp: TimeStamp, coord_func):
    # if param_row[['Wdot', 'e', 'sqrtA', 'M0', 'W0', 'w']].isna().any():
    if param_row.isna().any() or not param_row.exist:
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
