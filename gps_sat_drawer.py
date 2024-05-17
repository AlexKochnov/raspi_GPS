import numpy as np
import pandas as pd

from matplotlib import pyplot as plt
from math import sin, cos, radians

from pandas._typing import FilePath
from scipy.optimize import minimize
import pickle

import os

import Constants
from Transformations import aer2eci, aer2ecef, eci2ecef, Raer2ned, pm

# sat_df = pd.DataFrame(columns=['TOW', 'az', 'el', 'az_cor', 'el_cor'])

FLAG_01_CORRECTION = False


def calc_ubx_eci(row):
    # k_correction = 1 if FLAG_01_CORRECTION else 0
    return aer2eci(azim=row.azim,
                   elev=row.elev,
                   dist=row.dist,
                   lat=Constants.LLA[0],
                   lon=Constants.LLA[1],
                   # + row.TOW * Constants.OmegaEarthDot * 180 / np.pi * 0.1 * FLAG_01_CORRECTION,
                   alt=Constants.LLA[2],
                   time_sec=row.TOW)


def calc_ubx_ecef(row):
    # k_correction = 1 if FLAG_01_CORRECTION else 0
    return aer2ecef(azim=row.azim,
                    elev=row.elev,
                    dist=row.dist,
                    lat=Constants.LLA[0],
                    lon=Constants.LLA[1],
                    # + row.TOW * Constants.OmegaEarthDot * 180 / np.pi * 0.1 * FLAG_01_CORRECTION,
                    alt=Constants.LLA[2])
    # return pm.aer2ecef(row.azim, row.elev, row.dist,# *lla)
    #                    lla[0], lla[1] + row.TOW * OmegaEathDot * 180 / np.pi * 0.1, lla[2])


def calc_dist(row):
    r = Constants.ApproximateEarthRadius
    h = Constants.GPSAltitude
    if row.elev == -91:
        return np.nan
    dist = 0.5 * (
            np.sqrt(2) * np.sqrt(2 * h * h + 4 * h * r + r * r - r * r * np.cos(2 * row.elev * np.pi / 180)) -
            2 * r * np.sin(row.elev * np.pi / 180))
    return dist * 1000


def calc_dist3(row):
    aprox = calc_dist(row)

    def minimize_dist(dist):
        return abs(np.linalg.norm(aer2ecef(row.azim, row.elev, dist, *Constants.LLA)) - 5153 ** 2)

    solve = minimize(minimize_dist, aprox, tol=0.1, options={'disp': False, 'maxiter': 10})
    return solve.x


def calc_dist2(row):
    Re = Raer2ned(Constants.LLA[0], Constants.LLA[1], row.azim, row.elev)
    dist = 1 / (Re[0] ** 2 / Constants.a ** 2 + Re[1] ** 2 / Constants.a ** 2 + Re[2] ** 2 / Constants.b ** 2)
    return dist


def load_df(name: FilePath, H, time_correction_minus=False):
    df = pd.read_csv(
        # 'raw_2.txt',
        # 'sat_raw_calc_data1.txt',
        name,
        # 'sat2raw.log',
        sep=';', header=None,
        # names=['svId', 'gnssId', 'TOW', 'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z',
        #        'elev', 'azim', 'doMes', 'cpMes', 'prMes'])
        names=['svId', 'gnssId', 'TOW', 'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z',
               'elev', 'azim', 'doMes', 'cpMes', 'prMes',
               'alm_x1', 'alm_y1', 'alm_z1', 'eph_x1', 'eph_y1', 'eph_z1'])

    # df = df[df.gnssId == 'GNSS.GPS']
    # df = df[df.TOW > 50 * 3600]
    # print(min(df.TOW), max(df.TOW))
    # df[df.TOW < 50 * 3600].TOW += 3600 * 24 * 7
    # print(df.head())
    # df = df[df.TOW > (120 * 3600)]
    # df = df[df.TOW >= 0]
    if H:
        df = df[df.index % H == 0]
    df = df[df.TOW > 1]
    # df = df[df.TOW < 80 * 3600]
    # df.loc[df.TOW < 50 * 3600].TOW += 3600 * 24 * 7
    # df.loc[df['TOW'] > 50 * 3600, 'TOW'] -= 3600 * 24 * 7
    # df['time'] = df['TOW']# > 50 * 36003600 * 24 * 7
    # df.loc[df['time'] > 50 * 3600, 'time'] -= 3600 * 24 * 7
    df['time'] = df['TOW']
    if time_correction_minus:
        df.loc[df['TOW'] > 50 * 3600, 'TOW'] -= 3600 * 24 * 7

    # df.TOW += 3 * 3600

    # df = df[df.TOW > 120*3600]

    # print(min(df.TOW), max(df.TOW))
    # print(df[df.svId == 10].TOW)

    df.reset_index(drop=True, inplace=True)
    return df


if __name__ == "__main__":
    # step = 12
    # df1 = load_df('sat_raw_calc_data1.txt', step, time_correction_minus=True)
    # df2 = load_df('sat_raw_calc_data2.txt', step)
    # df3 = load_df('sat_raw_calc_data3.txt', step)
    # df = pd.concat([df1, df2, df3], ignore_index=True)
    # df = load_df('sat_raw_calc_data.txt', 5, time_correction_minus=True)

    # df = load_df('sat_raw_calc_data_60h.txt', 30)#, time_correction_minus=True)

    df = load_df('sat_raw_calc_data.txt', 5*60)#, time_correction_minus=True)

    # df = load_df('sat_raw_calc_data_1_big.txt', 0*10*60, time_correction_minus=True)  # , time_correction_minus=True)

    df.alm_x = df.alm_x1
    df.alm_y = df.alm_y1
    df.alm_z = df.alm_z1
    df.eph_x = df.eph_x1
    df.eph_y = df.eph_y1
    df.eph_z = df.eph_z1

    folder_path = 'satellites_xyz'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    print("DF is loaded")

    for svId in range(1, 33):
        sat = df[df.svId == svId]
        # sat = sat[sat.TOW > 10]
        if not len(sat):
            continue
        sat.reset_index(drop=True, inplace=True)

        # FLAG_01_CORRECTION = True

        sat['alm_ecef'] = sat.apply(
            lambda row: (row['alm_x'], row['alm_y'], row['alm_z'])
            , axis=1)
        # sat = sat.drop(columns=['alm_x', 'alm_y', 'alm_z'])
        # sat['alm_ecef'] = sat.apply(lambda row: eci2ecef(row['time'], *row['alm']), axis=1)
        sat['alm_lla'] = sat.apply(lambda row: pm.ecef2geodetic(*row['alm_ecef']), axis=1)
        # print(f'sv{svId}: alm calculated')

        sat['eph_ecef'] = sat.apply(lambda row: (row['eph_x'], row['eph_y'], row['eph_z']), axis=1)
        # sat = sat.drop(columns=['eph_x', 'eph_y', 'eph_z'])
        # sat['eph_ecef'] = sat.apply(lambda row: eci2ecef(row['time'], *row['eph']), axis=1)
        sat['eph_lla'] = sat.apply(lambda row: pm.ecef2geodetic(*row['eph_ecef']), axis=1)
        # print(f'sv{svId}: eph calculated')

        sat['dist'] = sat.apply(calc_dist, axis=1)
        # sat['ubx_ecef'] = sat.apply(calc_ubx_ecef, axis=1)
        # sat['ubx'] = sat.apply(calc_ubx_eci, axis=1)
        # sat['ubx'] = sat.apply(lambda row: ecef2eci(row['TOW'], *row['ubx_ecef']), axis=1)
        # print(f'sv{svId}: ubx calculated')
        # sat['ubx'] = sat.apply(calc_ubx_eci, axis=1)
        sat['ubx_ecef'] = sat.apply(calc_ubx_ecef,
                                    axis=1)  # sat.apply(lambda row: eci2ecef(row['TOW'], *row['ubx']), axis=1)
        sat['ubx_lla'] = sat.apply(lambda row: pm.ecef2geodetic(*row['ubx_ecef']), axis=1)

        # sat['ubx_ecef1'] = sat.apply(lambda row: eci2ecef(row['TOW']*1.1, *row['ubx']), axis=1)
        # sat.to_csv('sat12.csv', sep=';')
        # sat_df.to_csv(f'sat{svId}.csv', sep=';')
        # sat_df = pd.DataFrame(columns=['TOW', 'az', 'el', 'az_cor', 'el_cor'])

        # FLAG_01_CORRECTION = False

        # #K = 1
        # sat['alm1'] = sat.apply(lambda row: (row['alm_x1'], row['alm_y1'], row['alm_z1']), axis=1)
        # sat = sat.drop(columns=['alm_x1', 'alm_y1', 'alm_z1'])
        # sat['alm_ecef1'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['alm1']), axis=1)
        #
        # sat['eph1'] = sat.apply(lambda row: (row['eph_x1'], row['eph_y1'], row['eph_z1']), axis=1)
        # sat = sat.drop(columns=['eph_x1', 'eph_y1', 'eph_z1'])
        # sat['eph_ecef1'] = sat.apply(lambda row: eci2ecef(row['TOW'], *row['eph1']), axis=1)
        # sat['eph_lla1'] = sat.apply(lambda row: pm.ecef2geodetic(*row['eph_ecef1']), axis=1)

        # fig, axs = plt.subplots(3, figsize=(14, 8))
        fig, axs = plt.subplots(3, 2, figsize=(14, 8))
        print(axs.shape)
        htime = sat.TOW / 3600
        # htime = sat.time / 3600


        # serialized = pickle.dumps(sat)
        # with open('sat2bytes.pkl', 'wb') as f:
        #     pickle.dump(sat, f)
        # print('have serialized sat2bytes.pkl')

        # with open('sat6.bin', 'wb') as file:
        #     pickle.dump(sat, file)

        def plot_axs(i, j, y1, y2, y3, ylabel, k=1000, base=0):
            if axs.shape == (3, 1):
                ax = axs[i]
            else:
                ax = axs[i, j]
            ax.plot(htime, y3 / k - base / k, label='ubx', linestyle='-', linewidth=6, color='red',
                    alpha=0.35, zorder=-100)  # marker='o', markersize=4, linewidth=0.3)
            ax.plot(htime, y1 / k - base / k, label='alm', linestyle='-', linewidth=3, color='mediumaquamarine',
                    alpha=1, zorder=2)
            ax.plot(htime, y2 / k - base / k, label='eph', linestyle='--', linewidth=1.5, color='indigo',
                    alpha=1, zorder=3)
            # axs[i].plot(sat.TOW, y4, label='ubx1', linestyle='-')  # marker='o', markersize=4, linewidth=0.3)
            ax.set_ylabel(ylabel)

            # if i ==2:
            # ax.grid('both')
            # else:
            ax.grid()

            # ax.grid(which='minor')
            # ax.grid(which='major', linewidth=2)

            # ax.grid(True, which='major', linewidth=2)  # Большая сетка толщиной 2
            # ax.grid(True, which='minor', linestyle=':', linewidth=0.5)  # Маленькая сетка с базовой толщиной
            # ax.grid(which='both')

            # axs[i].axhline(y=ecef[i], color='r', linewidth=1, label='receiver')
            ax.legend(loc='upper right', facecolor='lightgrey')

            ymax = ax.get_ylim()[1]
            for t in np.arange(min(htime), max(htime), 12.0):
                ax.axvline(x=t, color='y', linewidth=1)
                # axs[i].annotate(f'{round(t - min(htime)): 2} ч', (t, ymax*0.95), color='y')
                if i != 2:
                    ax.text(t, ymax * 1.1, f'{round(t - min(htime)): 2} ч', color='y', ha='center')
            # if max(ax.get_ylim()) > 500:
            #     ax.set_ylim([-500, 500])
            # ax.grid(True, which='both')
            # ax.grid(True, which='major', linewidth=2)  # Большая сетка толщиной 2
            # ax.grid(True, which='minor', linewidth=1)  # Маленькая сетка с базовой толщиной




        for i in range(3):
            plot_axs(i, 0, sat.alm_lla.apply(lambda x: x[i]), sat.eph_lla.apply(lambda x: x[i]),
                     sat.ubx_lla.apply(lambda x: x[i]), ['Широта, градусы', 'Долгота, градусы', 'Высота, м'][i],
                     k=1)  # + ' ecef, K=1.1 км')#, sat.ubx_ecef.apply(lambda x: x[i]))  # , sat.alm.apply(lambda x: x[i]))
            # plot_axs(i, 1, sat.alm_lla.apply(lambda x: x[i]), sat.eph_lla.apply(lambda x: x[i]),
            #          sat.ubx_lla.apply(lambda x: x[i]), ['Широта, градусы', 'Долгота, градусы', 'Высота, м'][i], k=1, base=sat.ubx_lla.apply(lambda x: x[i]))

            # axs[i].plot(htime, sat.eph_lla1.apply(lambda x: x[i]), label='eph1', linestyle='--', linewidth=2, color='orange',
            #         zorder=3)
            # plot_axs(i, 0, sat.alm.apply(lambda x: x[i]), sat.eph.apply(lambda x: x[i]),
            #          sat.ubx.apply(lambda x: x[i]),
            #          'XYZ'[i] + ' ИСК, км')


            plot_axs(i, 1, sat.alm_ecef.apply(lambda x: x[i]), sat.eph_ecef.apply(lambda x: x[i]),
                     sat.ubx_ecef.apply(lambda x: x[i]),
                     'XYZ'[i] + ' ГЦК, км')  # , sat.alm_ecef.apply(lambda x: x[i]))
            # plot_axs(i, 1, sat.alm_ecef1.apply(lambda x: x[i]), sat.eph_ecef1.apply(lambda x: x[i]),
            #          sat.ubx_ecef1.apply(lambda x: x[i]),
            #          'XYZ'[i] + ' ecef, K=1, км')#, sat.ubx_ecef1.apply(lambda x: x[i]))

        # fig, axs = plt.subplots(3, figsize=(10, 8))
        # tows = [13, 27.368662225545556, 52.683471022215, 78]
        # for i in range(3):
        #     axs[i].plot(sat.TOW / 3600, sat.alm_lla.apply(lambda x: x[i]))
        #     axs[i].grid()
        #     axs[i].set_ylabel(f'{['Широта, градусы', 'Долгота, градусы', 'Высота, м'][i]}')
        #     for j in range(len(tows) - 1):
        #         # print(tows[j], tows[j+1])
        #         axs[i].axvspan(tows[j], tows[j + 1], alpha=0.15, color=['yellow', 'red', 'green', 'gray'][j],
        #                        label=f'Набор {j}')
        #     axs[i].legend(loc='upper right', facecolor='lightgrey')
        # plt.xlabel('Время TOW, ч')
        # fig.suptitle(f"Координаты спутника #{6}, вычисленные по альманах")
        # plt.tight_layout()
        # plt.savefig('ALM_lla.png', dpi=600)

        # plt.xlabel('Время TOW, ч')
        fig.suptitle(f"Координаты спутника #{svId}")
        plt.tight_layout()
        plt.savefig(os.path.join(folder_path,
                                 f'sat_lla{svId:02}.png'), dpi=500)

        # fig, axs = plt.subplots(3, figsize=(10, 8))
        # tows = [12, 22.368662225545556, 46.683471022215, 78]
        #
        # for i in range(3):
        #     axs[i].plot(sat.TOW / 3600, sat.alm_lla.apply(lambda x: x[i]))
        #     axs[i].grid()
        #     axs[i].set_ylabel(f'{['lat, °', 'lon, °', 'alt, км'][i]}')
        #     for j in range(len(tows) - 1):
        #         # print(tows[j], tows[j+1])
        #         axs[i].axvspan(tows[j], tows[j + 1], alpha=0.15, color=['yellow', 'red', 'green', 'gray'][j],
        #                        label=f'Набор {j}')
        # plt.xlabel('Время TOW, ч')
        # fig.suptitle(f"Координаты спутника #{6}, вычисленные по альманах")
        # plt.tight_layout()
        # plt.savefig('ALM_lla.png', dpi=600)

        # fig, axs = plt.subplots(3, figsize=(10, 8))
        # tows = [22.5, 25, 28, 46.5, 50.5, 52, 72, 73.5, 76]
        # for i in range(3):
        #     axs[i].plot(sat.TOW / 3600, sat.eph_lla.apply(lambda x: x[i]))
        #     axs[i].grid()
        #     axs[i].set_ylabel(f'{['Широта, градусы', 'Долгота, градусы', 'Высота, м'][i]}')
        #     # for tow in tows:
        #     #     for j in range(len(tows) - 1):
        #     #         # print(tows[j], tows[j+1])
        #     axs[i].axvspan(22.5, 24.7, alpha=0.15, label=f'Набор {1}', color='yellow')
        #     axs[i].axvspan(24.7, 28, alpha=0.15, label=f'Набор {2}', color='red')
        #     axs[i].axvspan(46.2, 50, alpha=0.15, label=f'Набор {3}', color='green')
        #     axs[i].axvspan(50, 52, alpha=0.15, label=f'Набор {4}', color='blue')
        #     axs[i].axvspan(72, 73.5, alpha=0.15, label=f'Набор {5}', color='gray')
        #     axs[i].axvspan(73.5, 76, alpha=0.15, label=f'Набор {6}', color='purple')
        #
        #     axs[i].legend(loc='upper right', facecolor='lightgrey')
        #     axs[i].set_xlim(20, 85)
        # plt.xlabel('Время TOW, ч')
        # fig.suptitle(f"Координаты спутника #{6}, вычисленные по альманах")
        # plt.tight_layout()
        # plt.savefig('EPH_lla.png', dpi=600)

        # plt.show()

        del sat
        print(f'sv {svId} is ready')
        pass

    pass
