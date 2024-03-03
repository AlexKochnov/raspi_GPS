import numpy as np

import pandas as pd

from matplotlib import pyplot as plt
import pymap3d as pm
import os

if __name__ == "__main__":
    df = pd.read_csv('sat_raw_calc_data.txt', sep=';', header=None,
                     names=['svId', 'gnssId', 'TOW', 'alm_x', 'alm_y', 'alm_z', 'eph_x', 'eph_y', 'eph_z', 'elev',
                            'azim', 'doMes', 'cpMes', 'prMes'])

    lla = (55.690555555555555, 37.858333333333334, 140)

    df = df[df.gnssId == 'GNSS.GPS']
    df.reset_index(drop=True, inplace=True)
    folder_path = 'satellites_xyz'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # df.TOW = (df.TOW -df.TOW.iloc[0]) / 3600
    df.TOW /= 3600
    for svId in range(1, 33):
        sat = df[df.svId == svId]
        sat.reset_index(drop=True, inplace=True)
        ECEF = np.array([(np.nan, np.nan, np.nan) if row.prMes is np.nan #or index % 500 != 0
                         else pm.aer2ecef(row.azim, row.elev, row.prMes, *lla) for index, row in sat.iterrows()])

        fig, axs = plt.subplots(3, 1, figsize=(8, 8))


        def plot_axs(i, y1, y2, y3, ylabel):
            axs[i].plot(sat.TOW, y1, label='alm', linestyle='-', linewidth=2)
            axs[i].plot(sat.TOW, y2, label='eph', linestyle='--', linewidth=2)
            axs[i].plot(sat.TOW, y3, label='ubx', linestyle='-')#marker='o', markersize=4, linewidth=0.3)
            axs[i].set_ylabel(ylabel)
            axs[i].grid()
            axs[i].legend(loc='upper right')


        plot_axs(0, sat.alm_x / 1000, sat.eph_x / 1000, ECEF[:, 0] / 1000, 'X, км')
        plot_axs(1, sat.alm_y / 1000, sat.eph_y / 1000, ECEF[:, 1] / 1000, 'Y, км')
        plot_axs(2, sat.alm_z / 1000, sat.eph_z / 1000, ECEF[:, 2] / 1000, 'Z, км')

        plt.xlabel('Время TOW, ч')
        fig.suptitle(f"Координаты спутника #{svId}")
        plt.tight_layout()
        plt.savefig(os.path.join(folder_path, f'sat{svId:02}.png'), dpi=600)
        plt.show()

        del sat

        pass

    pass
