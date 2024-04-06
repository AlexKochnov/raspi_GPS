import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import pymap3d as pm
import os
from math import radians, degrees, pi, asin, sin, cos, atan2

r = 6400
h = 20200
OmegaEathDot = 7.2921151467 * 10e-5

lla = [55.690555555555555, 37.858333333333334, 140]

def calc_ubx_eci(row):
    return pm.aer2ecef(row.azim, row.elev, row.dist,
                       lla[0], lla[1] - row.TOW * OmegaEathDot * 180 / np.pi * 0.9, lla[2])
def calc_ubx_ecef(row):
    def R(phi, lam):
        phi = np.radians(phi)
        lam = np.radians(lam)
        return np.array([
            [-sin(phi) * cos(lam), -sin(phi) * sin(lam), cos(phi)],
            [-sin(lam), cos(lam), 0],
            [-cos(phi) * cos(lam), -cos(phi) * sin(lam), -sin(phi)],
        ])
    LLA = (lla[0], lla[1] + row.TOW * OmegaEathDot * 180 / np.pi * 0.1, lla[2])
    az = radians(row.azim)
    el = radians(row.elev)
    E, N, U = [sin(az) * cos(el) * row.dist, cos(az) * cos(el) * row.dist, sin(el) * row.dist]
    NED = np.array([N, E, -U])
    UVW = np.transpose(R(LLA[0], LLA[1])) @ NED
    XYZ = pm.geodetic2ecef(*LLA)
    ECEF = UVW + XYZ
    return ECEF

    # return pm.aer2ecef(row.azim, row.elev, row.dist,# *lla)
    #                    lla[0], lla[1] + row.TOW * OmegaEathDot * 180 / np.pi * 0.1, lla[2])


def calc_dist(row):
    dist = 0.5 * (
            np.sqrt(2) * np.sqrt(2 * h * h + 4 * h * r + r * r - r * r * np.cos(2 * row.elev * np.pi / 180)) -
            2 * r * np.sin(row.elev * np.pi / 180))
    return dist * 1000


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
    df.reset_index(drop=True, inplace=True)

    folder_path = 'satellites_xyz'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    for svId in range(12, 33):
        sat = df[df.svId == svId]
        sat.reset_index(drop=True, inplace=True)

        sat['alm'] = sat.apply(lambda row: (row['alm_x'], row['alm_y'], row['alm_z']), axis=1)
        sat = sat.drop(columns=['alm_x', 'alm_y', 'alm_z'])
        sat['alm_ecef'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['alm']), axis=1)

        sat['eph'] = sat.apply(lambda row: (row['eph_x'], row['eph_y'], row['eph_z']), axis=1)
        sat = sat.drop(columns=['eph_x', 'eph_y', 'eph_z'])
        sat['eph_ecef'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['eph']), axis=1)

        sat['dist'] = sat.apply(calc_dist, axis=1)
        sat['ubx_ecef'] = sat.apply(calc_ubx_ecef, axis=1)
        sat['ubx'] = sat.apply(lambda row: ecef2eci(row['TOW'], *row['ubx_ecef']), axis=1)

        fig, axs = plt.subplots(3, 2, figsize=(14, 8))
        htime = sat.TOW / 3600

        def plot_axs(i, j, y1, y2, y3, ylabel, base=0):
            axs[i, j].plot(htime, y3 / 1000 - base/1000, label='ubx', linestyle='-', linewidth=6, color='red',
                           alpha=0.35, zorder=1)
            axs[i, j].plot(htime, y1 / 1000 - base/1000, label='alm', linestyle='-', linewidth=3, color='tab:blue',
                           zorder=2)
            axs[i, j].plot(htime, y2 / 1000 - base/1000, label='eph', linestyle='--', linewidth=2, color='yellow',
                           zorder=3)
            axs[i, j].set_ylabel(ylabel)
            axs[i, j].grid()
            axs[i, j].legend(loc='upper right', facecolor='lightgrey')

            ymax = axs[i, j].get_ylim()[1]
            for t in np.arange(min(htime), max(htime), 12.0):
                axs[i, j].axvline(x=t, color='y', linewidth=1)
                axs[i, j].text(t, ymax * 1.1, f'{round(t - min(htime)): 2} ч', color='y', ha='center')

        for i in range(3):
            plot_axs(i, 0, sat.alm.apply(lambda x: x[i]), sat.eph.apply(lambda x: x[i]),
                     sat.ubx.apply(lambda x: x[i]), 'XYZ'[i] + ' eci, км')
            plot_axs(i, 1, sat.alm_ecef.apply(lambda x: x[i]), sat.eph_ecef.apply(lambda x: x[i]),
                     sat.ubx_ecef.apply(lambda x: x[i]), 'XYZ'[i] + ' ecef, км')

        axs[0, 0].set_title('Координаты в ECI\n')
        axs[0, 1].set_title('Координаты в ECEF\n')
        plt.xlabel('Время TOW, ч')
        fig.suptitle(f"Координаты спутника #{svId}")
        plt.tight_layout()
        plt.savefig(os.path.join(folder_path, f'sat{svId:02}.png'), dpi=500)
        del sat
        pass
    pass
