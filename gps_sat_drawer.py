import numpy as np
import pandas as pd

from matplotlib import pyplot as plt

from datetime import datetime, timedelta

import pymap3d as pm

import os

r = 6400
h = 20200
OmegaEathDot = 7.2921151467 * 10e-5
start_date = datetime(1980, 1, 1) + timedelta(weeks=2048 + 256)

lla = (55.690555555555555, 37.858333333333334, 140)
ecef = (2842100.6406425796, 4164888.83911829, 3893127.006272498)  # 2800, 4200, 3900 км
ecef = np.array(ecef) / 1000


def calc_ubx_eci(row):
    dist = 0.5 * (
            np.sqrt(2) * np.sqrt(2 * h * h + 4 * h * r + r * r - r * r * np.cos(2 * row.elev * np.pi / 180)) -
            2 * r * np.sin(row.elev * np.pi / 180))
    # ubx.append(pm.aer2eci(row.azim, row.elev, dist * 1000, *lla, t=time))
    # ubx.append(pm.aer2ecef(row.azim, row.elev, dist * 1000, *lla1))
    # ubx.append(pm.ecef2eci(*pm.aer2ecef(row.azim, row.elev, dist * 1000, *lla1), time=time))
    # ubx.append(aer_to_ecef(*lla1, row.azim, row.elev, dist * 1000))

    # x, y, z = pm.aer2ecef(row.azim, row.elev, dist * 1000, *lla)
    # # ubx.append((x, y, z))
    # tU = OmegaEathDot * row.TOW * 3600
    # X = x * np.cos(tU) + y * np.sin(tU)
    # Y = - x * np.sin(tU) + y * np.cos(tU)
    # Z = z
    # ubx1.append((X, Y, Z))

    # pm.aer2ecef(row.azim, row.elev, row.prMes, *lla)
    return pm.aer2ecef(row.azim, row.elev, dist * 1000,
                       lla[0], lla[1] - row.TOW * OmegaEathDot * 180 / np.pi * 0.9, lla[2])


def calc_ubx_ecef(row):
    dist = 0.5 * (
            np.sqrt(2) * np.sqrt(2 * h * h + 4 * h * r + r * r - r * r * np.cos(2 * row.elev * np.pi / 180)) -
            2 * r * np.sin(row.elev * np.pi / 180))
    # return pm.aer2ecef(row.azim, row.elev, dist * 1000, *lla)
    return pm.aer2ecef(row.azim, row.elev, dist * 1000,
                       lla[0], lla[1] + row.TOW * OmegaEathDot * 180 / np.pi * 0.1, lla[2])


def rotateZ(x, y, z, ang):
    return (
        x * np.cos(ang) + y * np.sin(ang),
        - x * np.sin(ang) + y * np.cos(ang),
        z
    )


def ecef2eci(t, x, y, z):
    return rotateZ(x, y, z, t * OmegaEathDot)


def eci2ecef(t, x, y, z):
    return rotateZ(x, y, z, - t * OmegaEathDot)


if __name__ == "__main__":
    df = pd.read_csv('sat_raw_calc_data.txt', sep=';', header=None,
                     names=['svId', 'gnssId', 'TOW', 'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z',
                            'elev', 'azim', 'doMes', 'cpMes', 'prMes'])

    df = df[df.gnssId == 'GNSS.GPS']
    # print(len(df))
    # df = df.dropna(subset=df.columns.difference(['svId', 'gnssId', 'TOW']), how='all')
    # print(len(df))
    # df = df.head(500000)

    # df['alm'] = df.apply(lambda row: (row['alm_x'], row['alm_y'], row['alm_z']), axis=1)
    # df = df.drop(columns=['alm_x', 'alm_y', 'alm_z'])
    # df['eph'] = df.apply(lambda row: (row['eph_x'], row['eph_y'], row['eph_z']), axis=1)
    # df = df.drop(columns=['eph_x', 'eph_y', 'eph_z'])
    df.reset_index(drop=True, inplace=True)

    folder_path = 'satellites_xyz'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    for svId in range(2, 33):
        sat = df[df.svId == svId]
        sat.reset_index(drop=True, inplace=True)

        sat['alm'] = sat.apply(lambda row: (row['alm_x'], row['alm_y'], row['alm_z']), axis=1)
        sat = sat.drop(columns=['alm_x', 'alm_y', 'alm_z'])
        sat['alm_ecef'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['alm']), axis=1)

        sat['eph'] = sat.apply(lambda row: (row['eph_x'], row['eph_y'], row['eph_z']), axis=1)
        sat = sat.drop(columns=['eph_x', 'eph_y', 'eph_z'])
        sat['eph_ecef'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['eph']), axis=1)

        sat['ubx_ecef'] = sat.apply(calc_ubx_ecef, axis=1)
        sat['ubx'] = sat.apply(lambda row: ecef2eci(row['TOW'], *row['ubx_ecef']), axis=1)
        # sat['ubx'] = sat.apply(calc_ubx_eci, axis=1)
        # sat['ubx_ecef'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['ubx']), axis=1)

        fig, axs = plt.subplots(3, 2, figsize=(14, 8))
        htime = sat.TOW / 3600

        def plot_axs(i, j, y1, y2, y3, ylabel, base=0):
            axs[i, j].plot(htime, y3 / 1000 - base/1000, label='ubx', linestyle='-', linewidth=6, color='red', alpha=0.35,
                           zorder=1)  # marker='o', markersize=4, linewidth=0.3)
            axs[i, j].plot(htime, y1 / 1000 - base/1000, label='alm', linestyle='-', linewidth=3, color='tab:blue',
                           zorder=2)
            axs[i, j].plot(htime, y2 / 1000 - base/1000, label='eph', linestyle='--', linewidth=2, color='yellow',
                           zorder=3)
            # axs[i].plot(sat.TOW, y4, label='ubx1', linestyle='-')  # marker='o', markersize=4, linewidth=0.3)
            axs[i, j].set_ylabel(ylabel)
            axs[i, j].grid()
            # axs[i].axhline(y=ecef[i], color='r', linewidth=1, label='receiver')
            axs[i, j].legend(loc='upper right', facecolor='lightgrey')

            ymax = axs[i, j].get_ylim()[1]
            for t in np.arange(min(htime), max(htime), 12.0):
                axs[i, j].axvline(x=t, color='y', linewidth=1)
                # axs[i].annotate(f'{round(t - min(htime)): 2} ч', (t, ymax*0.95), color='y')
                axs[i, j].text(t, ymax * 1.1, f'{round(t - min(htime)): 2} ч', color='y', ha='center')


        for i in range(3):
            plot_axs(i, 0, sat.alm.apply(lambda x: x[i]), sat.eph.apply(lambda x: x[i]),
                     sat.ubx.apply(lambda x: x[i]), 'XYZ'[i] + ' eci, км')#, sat.alm.apply(lambda x: x[i]))
            plot_axs(i, 1, sat.alm_ecef.apply(lambda x: x[i]), sat.eph_ecef.apply(lambda x: x[i]),
                     sat.ubx_ecef.apply(lambda x: x[i]), 'XYZ'[i] + ' ecef, км')#, sat.alm_ecef.apply(lambda x: x[i]))

        axs[0, 0].set_title('Координаты в ECI\n')
        axs[0, 1].set_title('Координаты в ECEF\n')
        # plot_axs(1, sat.alm_y / 1000, sat.eph_y / 1000, sat.ubx[:, 1] / 1000, 'Y, км')
        # plot_axs(2, sat.alm_z / 1000, sat.eph_z / 1000, sat.ubx[:, 2] / 1000, 'Z, км')

        # plot_axs(0, sat.alm_x / 1000 - sat.alm_x / 1000, sat.eph_x / 1000 - sat.alm_x / 1000,
        #          ubx[:, 0] / 1000 - sat.alm_x / 1000, 'X, км')
        # plot_axs(1, sat.alm_y / 1000 - sat.alm_y / 1000, sat.eph_y / 1000 - sat.alm_y / 1000,
        #          ubx[:, 1] / 1000 - sat.alm_y / 1000, 'Y, км')
        # plot_axs(2, sat.alm_z / 1000 - sat.alm_z / 1000, sat.eph_z / 1000 - sat.alm_z / 1000,
        #          ubx[:, 2] / 1000 - sat.alm_z / 1000, 'Z, км')

        plt.xlabel('Время TOW, ч')
        fig.suptitle(f"Координаты спутника #{svId}")
        plt.tight_layout()
        plt.savefig(os.path.join(folder_path, f'sat{svId:02}.png'), dpi=500)
        # plt.show()

        del sat
        print(f'sv {svId} is ready')
        pass

    pass
