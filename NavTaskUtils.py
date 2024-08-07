import numpy as np
import pandas as pd

import Constants
import SatellitesCoordinateCalculator as SCC
import Transformations
from TimeStamp import TimeStamp


""" 
Для Storage.calc_navigation_task 
"""

def calc_alm_af_dt(ALM, time, N):
    N0a = ALM['week']
    Toa = ALM['Toa']
    af0 = ALM['af0']
    af1 = ALM['af1']
    tk = (N - N0a) * 604800 + time - Toa  # + 3600 * 6
    tk = SCC.check_gps_time(tk)
    # return af0
    af_dt = af0 + af1 * tk
    return af_dt

def calc_eph_af_dt(EPH, time, N):
    Toc = EPH['Toc']
    Toe = EPH['Toe']
    af2 = EPH['af2']
    af1 = EPH['af1']
    af0 = EPH['af0']

    tk = 0 * 604800 + time - Toe
    tk = SCC.check_gps_time(tk)
    # return af0
    af_dt = af0 + af1 * (tk - Toc) + af2 * (tk - Toc) ** 2
    return af_dt


def make_ae_nav_data(navigation_parameters, ephemeris_parameters, almanac_parameters, time_stamp, gps_flag=True):
    nav_cols = ['svId', 'gnssId', 'pr_stamp', 'prRMSer', 'prMes', 'prRes', 'prStedv', 'nav_score', 'alm_score',
                'eph_score']
    coord_cols = ['svId', 'gnssId', 'xyz_stamp', 'X', 'Y', 'Z', 'lat', 'lon', 'alt', 'azim', 'polar', 'radius',
                  'real_rho', 'Dt', 'af_dt']
    nav_data = pd.DataFrame(navigation_parameters.apply(calc_nav, axis=1).to_list(), columns=nav_cols)
    # eph_func = SCC.calc_gps_eph if gps_flag else else SCC.calc
    eph_coord_data = pd.DataFrame(
        ephemeris_parameters.apply(lambda row: calc_coords(row, time_stamp, SCC.calc_gps_eph, calc_eph_af_dt), axis=1).to_list(),
        columns=coord_cols)
    alm_coord_data = pd.DataFrame(
        almanac_parameters.apply(lambda row: calc_coords(row, time_stamp, SCC.calc_gps_alm, calc_alm_af_dt), axis=1).to_list(),
        columns=coord_cols)
    ephemeris_data = pd.merge(eph_coord_data, nav_data, on=['svId', 'gnssId']). \
        drop(columns=['alm_score']).rename(columns={'eph_score': 'coord_score'})
    ephemeris_data.Dt += ephemeris_data.prMes / Constants.c
    almanac_data = pd.merge(alm_coord_data, nav_data, on=['svId', 'gnssId']). \
        drop(columns=['eph_score']).rename(columns={'alm_score': 'coord_score'})
    almanac_data.Dt += almanac_data.prMes / Constants.c
    ephemeris_data['used'] = False
    almanac_data['used'] = False
    return ephemeris_data, almanac_data


def calc_nav_score(nav_row):
    quality = 2 if nav_row.qualityInd > 4 else (1 if nav_row.qualityInd == 4 else 0)
    score = (nav_row.health == 1) * (nav_row.visibility >= 2) * quality * (nav_row.prValid == True)
    if score:
        score *= 100*nav_row.cno / (abs(nav_row.prRes) + 1) / (5 + nav_row.prRMSer) #TODO: RMS error (not index)
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


def calc_coords(param_row, stamp: TimeStamp, coord_func, af_dt_func):
    # if param_row[['Wdot', 'e', 'sqrtA', 'M0', 'W0', 'w']].isna().any():
    if param_row.isna().any() or not param_row.exist:
        xyz = None
        af_dt = 0
    else:
        Txyz = coord_func(param_row, stamp.TOW, stamp.week)
        # af_dt = param_row.af0
        xyz = Txyz[1:]
        af_dt = Txyz[0]
        # af_dt = af_dt_func(param_row, stamp.TOW, stamp.week)
    if xyz:
        lla = Transformations.ecef2lla(*xyz)
        x, y, z = xyz
        r = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        azim, polar, radius = np.rad2deg(np.arctan2(y, x)), np.rad2deg(np.arccos(z/r)), r
    else:
        xyz = (np.nan, np.nan, np.nan)
        lla = (np.nan, np.nan, np.nan)
        azim, polar, radius = (np.nan, np.nan, np.nan)
    rho = np.linalg.norm(np.array(xyz) - np.array(Constants.ECEF))
    return param_row.svId, param_row.gnssId, stamp, *xyz, *lla, azim, polar, radius, rho, -rho / Constants.c + af_dt, af_dt


def calc_nav(nav_row):
    return (nav_row.svId, nav_row.gnssId, nav_row.receiving_stamp, nav_row.prRMSer, nav_row.prMes, nav_row.prRes,
            nav_row.prStedv, calc_nav_score(nav_row), calc_alm_score(nav_row), calc_eph_score(nav_row))
