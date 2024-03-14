import numpy as np
import pandas as pd

from matplotlib import pyplot as plt
from datetime import datetime, timedelta

import pymap3d as pm
from pyproj import Proj, transform

import os


def aer_to_ecef(observer_lat, observer_lon, observer_alt, az, elev, slant_range):
    # Параметры Земли
    a = 6378137.0  # Большая полуось (м)
    f = 1 / 298.257223563  # Плоскость (безразмерное число)
    b = (1 - f) * a  # Малая полуось (м)
    e = np.sqrt(1 - (b / a) ** 2)  # Эксцентриситет

    # Преобразование геодезических координат наблюдателя в ECEF
    N = a / np.sqrt(1 - e ** 2 * np.sin(np.radians(observer_lat)) ** 2)
    X = (N + observer_alt) * np.cos(np.radians(observer_lat)) * np.cos(np.radians(observer_lon))
    Y = (N + observer_alt) * np.cos(np.radians(observer_lat)) * np.sin(np.radians(observer_lon))
    Z = (N * (1 - e ** 2) + observer_alt) * np.sin(np.radians(observer_lat))

    # Преобразование AER координат в ECEF координаты объекта
    obj_X = X + slant_range * np.cos(np.radians(elev)) * np.sin(np.radians(az))
    obj_Y = Y + slant_range * np.cos(np.radians(elev)) * np.cos(np.radians(az))
    obj_Z = Z + slant_range * np.sin(np.radians(elev))

    return obj_X, obj_Y, obj_Z


if __name__ == "__main__":
    df = pd.read_csv('sat_raw_calc_data.txt', sep=';', header=None,
                     names=['svId', 'gnssId', 'TOW', 'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z',
                            'elev', 'azim', 'doMes', 'cpMes', 'prMes'])

    # ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    # lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    # pyproj.transform(lla, ecef, x, y, z, radians=False)
    r = 6400
    h = 20200
    OmegaEathDot = 7.2921151467 * 10e-5 * 0.9
    start_date = datetime(1980, 1, 1) + timedelta(weeks=2048 + 256)

    lla = (55.690555555555555, 37.858333333333334, 140)
    ecef = (2842100.6406425796, 4164888.83911829, 3893127.006272498)  # 2800, 4200, 3900 км
    ecef = np.array(ecef) / 1000

    df = df[df.gnssId == 'GNSS.GPS']
    # df = df.head(500000)
    df.reset_index(drop=True, inplace=True)
    folder_path = 'satellites_xyz'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # df.TOW = (df.TOW -df.TOW.iloc[0]) / 3600
    df.TOW /= 3600
    for svId in range(2, 33):
        sat = df[df.svId == svId]
        sat.reset_index(drop=True, inplace=True)
        ubx = []
        ubx1 = []
        for index, row in sat.iterrows():
            lla1 = (lla[0], lla[1] - row.TOW * 3600 * OmegaEathDot * 180 / np.pi, lla[2])
            dist = 0.5 * (
                    np.sqrt(2) * np.sqrt(2 * h * h + 4 * h * r + r * r - r * r * np.cos(2 * row.elev * np.pi / 180)) -
                    2 * r * np.sin(row.elev * np.pi / 180))
            time = start_date + timedelta(seconds=row.TOW * 3600)
            # if index % 100 != 0:
            #     ubx.append((np.nan, np.nan, np.nan))
            # else:
            # ubx.append(pm.aer2eci(row.azim, row.elev, dist * 1000, *lla, t=time))
            ubx.append(pm.aer2ecef(row.azim, row.elev, dist * 1000, *lla1))
            # ubx.append(aer_to_ecef(*lla1, row.azim, row.elev, dist * 1000))

            # x, y, z = pm.aer2ecef(row.azim, row.elev, dist * 1000, *lla)
            # # ubx.append((x, y, z))
            # tU = OmegaEathDot * row.TOW * 3600
            # X = x * np.cos(tU) + y * np.sin(tU)
            # Y = - x * np.sin(tU) + y * np.cos(tU)
            # Z = z
            # ubx1.append((X, Y, Z))

            # pm.aer2ecef(row.azim, row.elev, row.prMes, *lla)
        ubx = np.array(ubx)
        ubx1 = np.array(ubx1)
        # ubx = [np.array(ubx[:, 0]), np.array(ubx[:, 1]), np.array(ubx[:, 2])]

        fig, axs = plt.subplots(3, 1, figsize=(10, 8))


        def plot_axs(i, y1, y2, y3, ylabel):
            axs[i].plot(sat.TOW, y3, label='ubx', linestyle='-', linewidth=6, color='red', alpha=0.35,
                        zorder=1)  # marker='o', markersize=4, linewidth=0.3)
            axs[i].plot(sat.TOW, y1, label='alm', linestyle='-', linewidth=3, color='tab:blue', zorder=2)
            axs[i].plot(sat.TOW, y2, label='eph', linestyle='--', linewidth=2, color='yellow', zorder=3)
            # axs[i].plot(sat.TOW, y4, label='ubx1', linestyle='-')  # marker='o', markersize=4, linewidth=0.3)
            axs[i].set_ylabel(ylabel)
            axs[i].grid()
            # axs[i].axhline(y=ecef[i], color='r', linewidth=1, label='receiver')
            axs[i].legend(loc='upper right', facecolor='lightgrey')

            ymin = axs[i].get_ylim()[1]
            for t in np.arange(min(sat.TOW), max(sat.TOW), 12.0):
                axs[i].axvline(x=t, color='y', linewidth=1)
                # axs[i].annotate(f'{round(t - min(sat.TOW)): 2} ч', (t, ymin*0.95), color='y')
                axs[i].text(t, ymin * 1.1, f'{round(t - min(sat.TOW)): 2} ч', color='y', ha='center')


        # plot_axs(0, sat.alm_x / 1000, sat.eph_x / 1000, ubx[:, 0] / 1000,  'X, км')
        # plot_axs(1, sat.alm_y / 1000, sat.eph_y / 1000, ubx[:, 1] / 1000, 'Y, км')
        # plot_axs(2, sat.alm_z / 1000, sat.eph_z / 1000, ubx[:, 2] / 1000, 'Z, км')

        plot_axs(0, sat.alm_x / 1000 - sat.alm_x / 1000, sat.eph_x / 1000 - sat.alm_x / 1000,
                 ubx[:, 0] / 1000 - sat.alm_x / 1000, 'X, км')
        plot_axs(1, sat.alm_y / 1000 - sat.alm_y / 1000, sat.eph_y / 1000 - sat.alm_y / 1000,
                 ubx[:, 1] / 1000 - sat.alm_y / 1000, 'Y, км')
        plot_axs(2, sat.alm_z / 1000 - sat.alm_z / 1000, sat.eph_z / 1000 - sat.alm_z / 1000,
                 ubx[:, 2] / 1000 - sat.alm_z / 1000, 'Z, км')

        plt.xlabel('Время TOW, ч')
        fig.suptitle(f"Координаты спутника #{svId}")
        plt.tight_layout()
        plt.savefig(os.path.join(folder_path, f'sat{svId:02}.png'), dpi=500)
        # plt.show()

        del sat
        print(f'sv {svId} is ready')
        pass

    pass
