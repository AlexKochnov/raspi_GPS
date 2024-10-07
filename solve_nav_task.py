import concurrent.futures
from datetime import datetime

import numpy as np

import Constants
import Minimizing
import Settings
from GNSS import NavDataType
from Satellite import Satellite
from TimeStamp import TimeStamp

optimize_methods = {
    'LM': Minimizing.solve_navigation_task_LevMar,
    'SQP': Minimizing.solve_navigation_task_SLSQP,
    'TC': Minimizing.solve_navigation_task_TC,
    'DB': Minimizing.solve_navigation_task_DogBox,
    'TRF': Minimizing.solve_navigation_task_TRF,
}


def calc_GDOP(satellites, position):
    X, Y, Z, cdt = position
    def calc_row(sat):
        dx, dy, dz = sat['X'] - X, sat['Y'] - Y, sat['Z'] - Z
        # R = np.sqrt(dx**2 + dy**2 + dz**2)
        R = sat['prMesCorrected'] - cdt
        return np.array([dx/R, dy/R, dz/R, 1])
    G = np.array([calc_row(sat) for sat in satellites])
    Qinv = G.T @ G
    Q = np.linalg.inv(Qinv)
    # Gx, Gy, Gz, Gt = (Q[i, i] for i in range(4))
    # PDOP = np.sqrt(Gx + Gy + Gz)
    # TDOP = np.sqrt(Gt)
    return float(np.sqrt(np.trace(Q)))


def solve_nav_task(satellites: list[Satellite], rcvTow, week, dataType: NavDataType, **kwargs):
    sats = []
    for sat in satellites:
        di = sat.get_calculation_dict(rcvTow, week, dataType)
        if di:
            di['prMesCorrected'] = di['prMes'] + Constants.c * di['af_dt']
            sats.append(di)
    sats.sort(key=lambda sat: sat['nav_score'], reverse=True)
    sats = sats[:Settings.MaximumMinimizingSatellitesCount]

    data = [[sat['X'], sat['Y'], sat['Z'], sat['prMesCorrected']] for sat in sats]
    data = np.array(data).T

    method = Settings.used_method
    method_func = optimize_methods[method]
    SOLVE = {
        'calc_stamp': TimeStamp(TOW=rcvTow, week=week),
        'sat_count': len(sats),
        'success': False,
        'fval': np.inf,
        'GDOP': 100,
        'result': None,
        'method': method
    }
    if SOLVE['sat_count'] < Settings.MinimumMinimizingSatellitesCount:
        return SOLVE

    def get_solve_line():
        t = datetime.now(tz=Constants.tz_utc)
        res = method_func(data)
        solve = {
            'result': res.x,
            'fval': np.linalg.norm(np.array(res.fun)),
            'success': res.success,
            'calc_time': (datetime.now(tz=Constants.tz_utc) - t).total_seconds(),
        }
        return solve | {'method': method} | {'GDOP': calc_GDOP(sats, res.x)}

    with concurrent.futures.ThreadPoolExecutor() as executor:
        future = executor.submit(get_solve_line)
        try:
            SOLVE |= future.result(timeout=Settings.max_calc_time)
        except concurrent.futures.TimeoutError:
            SOLVE |= {'calc_time': np.inf}
        except Exception as e:
            SOLVE |= {'calc_time': np.nan}

    return SOLVE
