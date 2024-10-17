import numpy as np
import pandas as pd

from Utils import Constants, Transformations
from old_code import SatellitesCoordinateCalculator as SCC
from Utils.TimeStamp import TimeStamp


""" 
Для Storage.calc_navigation_task 
"""


def make_ae_nav_data(navigation_parameters, ephemeris_parameters, almanac_parameters, time_stamp, rcvTOW, gps_flag=True):
    nav_cols = ['svId', 'gnssId', 'pr_stamp', 'prRMSer', 'prMes', 'prRes', 'prStedv', 'nav_score', 'alm_score',
                'eph_score']
    coord_cols = ['svId', 'gnssId', 'xyz_stamp', 'X', 'Y', 'Z', 'lat', 'lon', 'alt', 'azim', 'polar', 'radius',
                  'real_rho', 'Dt', 'af_dt']
    nav_data = pd.DataFrame(navigation_parameters.apply(calc_nav, axis=1).to_list(), columns=nav_cols)
    # eph_func = SCC.calc_gps_eph if gps_flag else else SCC.calc
    # T = TimeStamp(TOW=rcvTOW, week=time_stamp.week)
    # print(T)
    # nav_data.pr_stamp = nav_data.pr_stamp.apply(lambda st: T if st == time_stamp else st)
    # print(nav_data.pr_stamp)
    eph_coord_data = pd.DataFrame(
        ephemeris_parameters.apply(lambda row: calc_coords(row, time_stamp, rcvTOW, SCC.calc_gps_eph), axis=1).to_list(),
        columns=coord_cols)
    alm_coord_data = pd.DataFrame(
        almanac_parameters.apply(lambda row: calc_coords(row, time_stamp, rcvTOW, SCC.calc_gps_alm), axis=1).to_list(),
        columns=coord_cols)
    ephemeris_data = pd.merge(eph_coord_data, nav_data, on=['svId', 'gnssId']). \
        drop(columns=['alm_score']).rename(columns={'eph_score': 'coord_score'})
    ephemeris_data.Dt += ephemeris_data.prMes / Constants.c
    almanac_data = pd.merge(alm_coord_data, nav_data, on=['svId', 'gnssId']). \
        drop(columns=['eph_score']).rename(columns={'alm_score': 'coord_score'})
    almanac_data.Dt += almanac_data.prMes / Constants.c
    ephemeris_data['used'] = False
    almanac_data['used'] = False
    # almanac_data['coord_score'] = abs(almanac_data['week'] - time_stamp.week) <= 1
    # ephemeris_data['coord_score'] = abs(ephemeris_data['week'] - time_stamp.week) <= 1
    return ephemeris_data, almanac_data


def get_scores(data, *args, BiggerBetter=True):
    score = 0
    if BiggerBetter:
        for arg in sorted(args):
            if data >= arg:
                score += 1
    else:
        for arg in sorted(args)[::-1]:
            if data <= arg:
                score += 1
    return score


def calc_nav_score(nav_row):
    # quality = 2 if nav_row.qualityInd > 4 else (1 if nav_row.qualityInd == 4 else 0)
    # score = (nav_row.health == 1) * (nav_row.visibility >= 2) * quality * (nav_row.prValid == True)
    # if score:
    #     score *= 100*nav_row.cno / (abs(nav_row.prRes) + 1) / (5 + nav_row.prRMSer) #TODO: RMS error (not index)
    # return score
    S_quality = get_scores(nav_row.qualityInd, 4, 5)
    S_cno = get_scores(nav_row.cno, 20, 30)
    S_prRes = get_scores(abs(nav_row.prRes), 10, 40, BiggerBetter=False) #+ 1
    S_prRMSer = get_scores(nav_row.prRMSer, 10, 40, BiggerBetter=False)
    S_prStedv = get_scores(nav_row.prStedv, 10, 40, BiggerBetter=False)
    S_visibility = get_scores(nav_row.visibility, 2, 3)
    if (nav_row.health == 1) and (nav_row.prValid == True):
        return S_quality * S_cno * S_prRes * S_prRMSer * S_prStedv * S_visibility
    return 0


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


def calc_coords(param_row, stamp: TimeStamp, rcvTOW, coord_func):
    # if param_row[['Wdot', 'e', 'sqrtA', 'M0', 'W0', 'w']].isna().any():
    if param_row.isna().any() or not param_row.exist:
        xyz = None
        af_dt = 0
    else:
        Txyz = coord_func(param_row, rcvTOW, stamp.week)
        # af_dt = param_row.af0
        if Txyz:
            xyz = Txyz[1:]
            af_dt = Txyz[0]
        else:
            xyz = None
            af_dt = 0
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
    return (param_row.svId, param_row.gnssId, TimeStamp(TOW=rcvTOW, week=stamp.week), *xyz, *lla, azim, polar, radius,
            rho, -rho / Constants.c + af_dt, af_dt)


def calc_nav(nav_row):
    return (nav_row.svId, nav_row.gnssId, nav_row.rcvTOW, nav_row.prRMSer, nav_row.prMes, nav_row.prRes,
            nav_row.prStedv, calc_nav_score(nav_row), calc_alm_score(nav_row), calc_eph_score(nav_row))
