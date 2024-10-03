import numpy as np

import Settings


class StorageColumnsLord:
    NAV_ORB_columns = {'health': np.int8, 'visibility': np.int8, 'ephUsability': np.int8, 'ephSource': np.int8,
                       'almUsability': np.int8, 'almSource': np.int8}
    NAV_SAT_columns = {'cno': np.int8, 'elev': np.int8, 'azim': np.int16, 'prRes': np.float32, 'qualityInd': np.int8,
                       'svUsed': np.bool_, 'health': np.int8, 'diffCorr': np.bool_, 'smoothed': np.bool_,
                       'orbitSourse': np.int8, 'ephAvail': np.bool_, 'almAvail': np.bool_}
    RXM_RAWX_columns = {'prMes': np.float64, 'cpMes': np.float64, 'doMes': np.float64, 'rcvTOW': np.float64,
                        'freqId': np.int8, 'locktime': np.uint16, 'cno': np.int8,
                        'prStedv': np.float32, 'cpStedv': np.float32, 'doStedv': np.float32,
                        'prValid': np.bool_, 'cpValid': np.bool_, 'halfCyc': np.bool_, 'subHalfCyc': np.bool_}
    # RXM_SVSI_columns = {'azim': np.int16, 'elev': np.int8, 'ura': np.int8, 'healthy': np.bool_, 'ephVal': np.bool_,
    #                     'almVal': np.bool_, 'notAvail': np.bool_, 'almAge': np.int8, 'ephAge': np.int8}
    RXM_MEASX_columns = {'mpathIndic': np.int8, 'dopplerMS': np.float64, 'dopplerHz': np.float64,
                         'wholeChips': np.int16, 'fracChips': np.int16, 'codePhase': np.float64,
                         'intCodePhase': np.int8, 'pseuRangeRMSErr': np.int8, 'prRMSer': np.float16}
    EPH_columns = {'health': np.int8, 'accuracy': np.int8,
                   'week': np.int16, 'Toe': np.int32, 'Toc': np.int32, 'Tgd': np.float64,  'Wdot': np.float64,
                   'dn': np.float64, 'i0': np.float64, 'IDOT': np.float64, 'e': np.float64, 'sqrtA': np.float64,
                   'M0': np.float64, 'W0': np.float64, 'w': np.float64,
                   'Crs': np.float64, 'Crc': np.float64, 'Cus': np.float64, 'Cuc': np.float64, 'Cis': np.float64,
                   'Cic': np.float64,
                   'af2': np.float64, 'af1': np.float64, 'af0': np.float64,
                   'IODE1': np.int16, 'IODE2': np.int16, 'IODC': np.int16}
    ALM_columns = {'health': np.int8, 'Data_ID': np.int8,
                   'week': np.int16, 'Toa': np.int32, 'e': np.float32, 'delta_i': np.float32, 'Wdot': np.float32,
                   'sqrtA': np.float32, 'W0': np.float32, 'w': np.float32, 'M0': np.float32, 'af0': np.float32,
                   'af1': np.float32, }
    # TODO: delete error
    solves_columns = {'success': np.bool_, 'X': np.float64, 'Y': np.float64, 'Z': np.float64,
                      'lat': np.float64, 'lon': np.float64, 'alt': np.float64,
                      'cdt': np.float64, 'dt': np.float64, 'GDOP': np.float64,
                      'fval': np.float64, 'error': np.float64, 'calc_time': np.float32}

    stamp_columns = {'svId': np.int8, 'gnssId': object}
    param_columns = stamp_columns | {'receiving_stamp': object, 'exist': np.bool_, 'is_old': np.bool_}
    general_nav_cols = stamp_columns | {name: object for name in ['receiving_stamp', 'NAV_ORB_stamp', 'RXM_RAWX_stamp',
                                                                  'NAV_SAT_stamp', 'RXM_MEASX_stamp']}
    # TODO: delete real_rho & Dt
    data_columns = stamp_columns | {'xyz_stamp': object, 'pr_stamp': object,
                                    'X': np.float64, 'Y': np.float64, 'Z': np.float64,
                                    'lat': np.float64, 'lon': np.float64, 'alt': np.float64,
                                    'azim': np.float32, 'polar': np.float32, 'radius': np.float32,
                                    'prRMSer': np.float16, 'prMes': np.float64, 'prRes': np.float32,
                                    'prStedv': np.float32, 'real_rho': np.float64, 'Dt': np.float64, 'af_dt': np.float64,
                                    'coord_score': np.float16, 'nav_score': np.float16, 'used': np.bool_}
    # TODO: delete one optimization method
    full_solves_columns = {'calc_stamp': object, 'sat_count': int, 'method': np.str_} | solves_columns
                          # {f'{Settings.using_methods[0]}_{name}': type for name, type in solves_columns.items()}
    # {f'{"SQP"}_{name}': type for name, type in solves_columns.items()} |\
    # {f'{"TRF"}_{name}': type for name, type in solves_columns.items()} | \
    # {f'{"TC"}_{name}': type for name, type in solves_columns.items()} | \
    # {f'{"DB"}_{name}': type for name, type in solves_columns.items()}
    # {f'{"COBYLA"}_{name}': type for name, type in solves_columns.items()}
    # {f'{"CF"}_{name}': type for name, type in solves_columns.items()} |\
    full_eph_columns = param_columns | EPH_columns
    full_alm_columns = param_columns | ALM_columns
    full_nav_columns = stamp_columns | {'receiving_stamp': object} | general_nav_cols | \
                       NAV_ORB_columns | NAV_SAT_columns | RXM_RAWX_columns | RXM_MEASX_columns
