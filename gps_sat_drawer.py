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

### Спизжено: https://github.com/fogleman/GPS
# import pg
from math import radians, degrees, pi, asin, sin, cos, atan2

EARTH_RADIUS = 6371
ALTITUDE = 20200

ZNEAR = 1
ZFAR = 1000000


def to_xyz(lat, lng, elevation, azimuth, altitude=ALTITUDE):
    r1 = EARTH_RADIUS
    r2 = r1 + altitude
    aa = radians(elevation) + pi / 2
    ar = asin(r1 * sin(aa) / r2)
    ad = pi - aa - ar
    angle = pi / 2 - ad
    x = cos(angle) * r2
    z = sin(angle) * r2
    # matrix = pg.Matrix()
    # matrix = matrix.rotate((0, 0, -1), pi / 2 - radians(azimuth))
    # matrix = matrix.rotate((-1, 0, 0), -radians(lat))
    # matrix = matrix.rotate((0, -1, 0), radians(lng))
    # return matrix * (x, 0, z)
    # Создаем матрицы поворота
    rot_z = np.array([[cos(pi / 2 - radians(azimuth)), -sin(pi / 2 - radians(azimuth)), 0],
                      [sin(pi / 2 - radians(azimuth)), cos(pi / 2 - radians(azimuth)), 0],
                      [0, 0, 1]])

    rot_x = np.array([[1, 0, 0],
                      [0, cos(-radians(lat)), -sin(-radians(lat))],
                      [0, sin(-radians(lat)), cos(-radians(lat))]])

    rot_y = np.array([[cos(radians(lng)), 0, sin(radians(lng))],
                      [0, 1, 0],
                      [-sin(radians(lng)), 0, cos(radians(lng))]])

    # Применяем повороты к вектору (x, 0, z)
    rotated_vector = np.dot(rot_z, np.dot(rot_x, np.dot(rot_y, np.array([x, 0, z]))))
    return rotated_vector * 1000


###

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



def R(phi, lam):
    return np.array([
        [-sin(phi)*cos(lam), -sin(phi)*sin(lam), 0],
        [-sin(lam), cos(lam), 0],
        [-cos(phi) * cos(lam), -cos(phi) * sin(lam), -sin(phi)],
    ])
def calc_ubx_ecef(row):

    # return pm.aer2ecef(row.azim, row.elev, dist * 1000, *lla)
    # if row.TOW % (3600*24) == 46350:
    #     a=0
    # if not np.isnan(row.dist):
    #     aer0 = (row.azim, row.elev, row.dist)
    #     xyz = pm.aer2ecef(*aer0, lla[0], lla[1] + row.TOW * OmegaEathDot * 180 / np.pi * 0.1, lla[2])
    #     aer = pm.ecef2aer(*xyz, *lla)
    #     with open('3.txt', 'a') as file:
    #         print(row.TOW, aer0[0], aer0[1], aer[0], aer[1], sep=';', file=file)
        # print(*aer0)
        # print(*aer)
        # dist1 = 0.5 * (
        #         np.sqrt(2) * np.sqrt(2 * h * h + 4 * h * r + r * r - r * r * np.cos(2 * aer[1] * np.pi / 180)) -
        #         2 * r * np.sin(aer[1] * np.pi / 180))
        # print(dist1)
    # return to_xyz(lla[0], lla[1], row.elev, row.azim)

    az = radians(row.azim)
    el = radians(row.elev)
    E,N,U = [sin(az) * cos(el) * row.dist, cos(az) * cos(el) * row.dist, sin(el) * row.dist]
    NED = np.array([N, E, -U])

    XYZ = pm.geodetic2ecef(lla[0], lla[1] - 0 * row.TOW * OmegaEathDot * 180 / np.pi * 0.1, lla[2])

    # xyz = pm.aer2ecef(row.azim, row.elev, row.dist, *lla)
    Pecef = + XYZ
    return (Pecef[0,1], Pecef[0,1], Pecef[0,2])

    # if not np.isnan(row.elev)
    #     print(f'{row.TOW}; {row.azim}; {row.elev}; {row.dist}; {xyz[0]}; {xyz[1]}; {xyz[2]}')
    return pm.aer2ecef(row.azim, row.elev, row.dist, *lla)
                       # lla[0], lla[1] + row.TOW * OmegaEathDot * 180 / np.pi * 0.1 * 0, lla[2])

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
    # df = pd.read_csv('sat_raw_calc_data.txt', sep=';', header=None,
    #                  names=['svId', 'gnssId', 'TOW', 'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z',
    #                         'elev', 'azim', 'doMes', 'cpMes', 'prMes'])
    df = pd.read_csv('sv12.csv', sep=';')

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

    # for svId in range(2, 33):
    for svId in range(12, 13):
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
        # sat['ubx'] = sat.apply(calc_ubx_eci, axis=1)
        # sat['ubx_ecef'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['ubx']), axis=1)
        sat.to_csv('sat12.csv', sep=';')

        fig, axs = plt.subplots(3, 2, figsize=(14, 8))
        htime = sat.TOW / 3600


        def plot_axs(i, j, y1, y2, y3, ylabel, base=0):
            axs[i, j].plot(htime, y3 / 1000 - base / 1000, label='ubx', linestyle='-', linewidth=6, color='red',
                           alpha=0.35,
                           zorder=1)  # marker='o', markersize=4, linewidth=0.3)
            axs[i, j].plot(htime, y1 / 1000 - base / 1000, label='alm', linestyle='-', linewidth=3, color='tab:blue',
                           zorder=2)
            axs[i, j].plot(htime, y2 / 1000 - base / 1000, label='eph', linestyle='--', linewidth=2, color='yellow',
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
                     sat.ubx.apply(lambda x: x[i]), 'XYZ'[i] + ' eci, км')  # , sat.alm.apply(lambda x: x[i]))
            plot_axs(i, 1, sat.alm_ecef.apply(lambda x: x[i]), sat.eph_ecef.apply(lambda x: x[i]),
                     sat.ubx_ecef.apply(lambda x: x[i]),
                     'XYZ'[i] + ' ecef, км')  # , sat.alm_ecef.apply(lambda x: x[i]))

        axs[0, 0].set_title('Координаты в ECI (абсолютной)\n')
        axs[0, 1].set_title('Координаты в ECEF (относительной)\n')
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
