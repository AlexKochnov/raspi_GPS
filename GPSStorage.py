from datetime import timedelta
import pickle

import numpy as np
from scipy.optimize import minimize
# from tabulate import tabulate

from GPSUtils import *
from UBXUnpacker import *

import Constants
import Minimizing
import SatellitesCoordinateCalculator as SCC
from Transformations import *


class GPSStorage:
    min_sat_using = 4
    max_sat_using = 8

    sat_calc_file = 'sat_raw_calc_data.txt'
    calc_log_file = 'calculations.log'
    # EPH_headers = ['SV_ID', 'week', 'Toe', 'Toc', 'IODE1', 'IODE2', 'IODC', 'IDOT', 'Wdot', 'Crs', 'Crc', 'Cus', 'Cuc',
    #                'Cis', 'Cic', 'dn', 'i0', 'e', 'sqrtA', 'M0', 'W0', 'w', 'Tgd', 'af2', 'af1', 'af0',
    #                'health', 'accuracy', 'receiving_time']
    # ALM_headers = ['SV_ID', 'week', 'Toa', 'e', 'delta_i', 'Wdot', 'sqrtA', 'W0', 'w', 'M0', 'af0', 'af1',
    #                'health', 'Data_ID', 'receiving_time']

    satellites: dict[(GNSS, int): Satellite] = dict()

    start_age = datetime(1980, 1, 6)
    start_week: datetime
    week = 0
    TOW = 0
    iTOW = 0
    fTOW = 0
    time: datetime

    ecefX, ecefY, ecefZ, pAcc = 0, 0, 0, 0
    ecefVX, ecefVY, ecefVZ, sAcc = 0, 0, 0, 0
    leapS = 0
    Valid = {'TOW': 0, 'week': 0, 'leapS': 0}

    MyCoordinatesSolve = None

    def __init__(self):
        pass

    def check_satellite(self, gnssId, svId):
        sat = (gnssId, svId)
        if sat not in self.satellites:
            self.satellites[sat] = Satellite(gnssId, svId)

    def update(self, data):
        # print(self.satellites)
        if data is None or not data:
            return
        if not isinstance(data, Message):
            return
        if hasattr(data, 'iTOW'):
            if self.iTOW != data.iTOW:
                self.end_of_tick()
                pass
            self.iTOW = data.iTOW
        if isinstance(data, AID_ALM):
            self.check_satellite(GNSS.GPS, data.svId)
            self.satellites[(GNSS.GPS, data.svId)].alm = data.alm
        if isinstance(data, AID_EPH):
            self.check_satellite(GNSS.GPS, data.svId)
            self.satellites[(GNSS.GPS, data.svId)].eph = data.eph
        if isinstance(data, NAV_POSECEF):
            self.ecefX = data.ecefX
            self.ecefY = data.ecefY
            self.ecefZ = data.ecefZ
            self.pAcc = data.pAcc
        if isinstance(data, NAV_VELECEF):
            self.ecefVX = data.ecefVX
            self.ecefVY = data.ecefVY
            self.ecefVZ = data.ecefVZ
            self.sAcc = data.sAcc
        if isinstance(data, NAV_TIMEGPS):
            self.week = data.week
            self.fTOW = data.fTOW
            self.TOW = data.TOW
            self.leapS = data.leapS
            self.Valid = data.Valid
            self.start_week = self.start_age + timedelta(self.week * 7)
            self.time = self.start_week + timedelta(seconds=self.TOW / 1000)
        if isinstance(data, NAV_SAT | NAV_ORB | RXM_SVSI | RXM_RAWX):
            for (gnssId, svId), param in getattr(data, data.attr).items():
                self.check_satellite(GNSS(gnssId), svId)
                setattr(self.satellites[(GNSS(gnssId), svId)], data.attr, param)
        if isinstance(data, RXM_SFRBX):
            pass

    def __str__(self):
        return str(self.__dict__)

    def get_TOW(self):
        if self.TOW:
            return self.TOW
        if self.iTOW:
            return self.iTOW / 1000
        return None

    def end_of_tick(self):
        self.calc_coordinates()
        self.satellite_logger()
        self.reset_variables()

    def calc_coordinates(self):
        for svId, satellite in self.satellites.items():
            self.satellites[svId].alm_coord = SCC.calc_sat_alm(satellite.alm, self.get_TOW(), self.week)
            self.satellites[svId].eph_coord = SCC.calc_sat_eph(satellite.eph, self.get_TOW(), self.week)

        good_eph = [satellite for satellite in self.satellites.values()
                    if satellite.eph is not None and satellite.rawx and satellite.eph_coord is not None]
                    # and satellite.rawx.prValid and satellite.rawx.cpValid]# is not None and satellite.sat.qualityInd > 3]
        if good_eph:
            a = 0
        Flag = False

        if Flag:
            # with open(self.calc_log_file + '5', 'rb') as logger:
            with open('5_calculation579460.000205107.log', 'rb') as logger:
                good_eph = pickle.load(logger)
        # good_eph.sort(key=lambda satellite: satellite.sat.qualityInd, reverse=True)
        good_eph_count = min(self.max_sat_using, len(good_eph))
        good_eph = good_eph[:good_eph_count]
        print(f'good_eph [{len(good_eph)}]: {good_eph}')
        if len(good_eph) < self.min_sat_using:
            self.MyCoordinatesSolve = None
            return
        serialized_good_eph = pickle.dumps(good_eph)
        print('serialized good eph:')
        print(serialized_good_eph)

        serialized_Storage = pickle.dumps(self)
        with open(f'storages/storage{self.TOW}.log', 'wb') as logger:
            logger.write(serialized_Storage)
        print('Serialized Storage:')
        print(serialized_Storage)

        if not Flag:
            with open(f'calculations/calculation{self.TOW}.log', 'wb') as logger:
                logger.write(serialized_good_eph)
            with open('parsed.log', 'a') as logger:
                logger.write(f'Serialized good eph [{len(good_eph)}]: {serialized_good_eph}\n')

        solve = Minimizing.solve_navigation_task(good_eph)
        xyz = solve.x[:3]
        ecef = eci2ecef(self.TOW, *xyz)
        lla = eci2lla(self.TOW, *xyz)
        # print(np.linalg.norm(np.array(xyz)))
        # print(np.linalg.norm(np.array(ecef)))
        print(f'ECI xyz and dT: {solve.x}')
        print(f'ECEF xyz: {ecef}')
        print(f'LLA calculated:{lla}')
        print(f'LLA my valid: {Constants.LLA}')
        error = np.linalg.norm(np.array(ecef) - np.array(Constants.ECEF))
        print(f'Full error: {error}')

        with open('parsed.log', 'a') as logger:
            logger.write(f'Solve: {solve}\n')
            logger.write(f'ECEF SOLVE: {ecef}\n')
            logger.write(f'LLA SOLVE: {lla}\n')
            logger.write(f'Full error: {error}\n')
            logger.write(f'DATA: {[*lla, error, self.TOW]}\n')

        self.MyCoordinatesSolve = solve
        if error < 1000:
            print("GOOD_ERROR")
            a = 0
        print(f'Calced coordinates {solve}')

    def satellite_logger(self):
        for svId, satellite in self.satellites.items():
            try:
                if satellite.alm_coord:
                    # alm_x, alm_y, alm_z, *_ = SCC.calc_sat_alm(satellite.alm, self.iTOW / 1000, self.week)
                    alm_x, alm_y, alm_z, *_ = satellite.alm_coord
                    alm_x1, alm_y1, alm_z1, *_ = SCC.calc_sat_alm(satellite.alm, self.iTOW / 1000, self.week, 1.0)

                else:
                    alm_x, alm_y, alm_z = np.nan, np.nan, np.nan
                    alm_x1, alm_y1, alm_z1 = np.nan, np.nan, np.nan

                if satellite.eph_coord:
                    # eph_x, eph_y, eph_z, *_ = SCC.calc_sat_eph(satellite.eph, self.iTOW / 1000, self.week)
                    eph_x, eph_y, eph_z, *_ = satellite.eph_coord
                    eph_x1, eph_y1, eph_z1, *_ = SCC.calc_sat_eph(satellite.eph, self.iTOW / 1000, self.week, 1.0)

                else:
                    eph_x, eph_y, eph_z = np.nan, np.nan, np.nan
                    eph_x1, eph_y1, eph_z1 = np.nan, np.nan, np.nan

                if satellite.sat:
                    elev = satellite.sat.elev
                    azim = satellite.sat.azim
                else:
                    elev, azim = np.nan, np.nan

                if satellite.rawx:
                    cpMes = satellite.rawx.cpMes
                    prMes = satellite.rawx.prMes
                    doMes = satellite.rawx.doMes
                else:
                    cpMes, prMes, doMes = np.nan, np.nan, np.nan
                with open(self.sat_calc_file, 'a') as file:
                    file.write(
                        f"{satellite.svId};{satellite.gnssId};{self.iTOW / 1000};{alm_x};{alm_y};{alm_z};{eph_x};{eph_y};{eph_z};{elev};{azim};{doMes};{cpMes};{prMes};{alm_x1};{alm_y1};{alm_z1};{eph_x1};{eph_y1};{eph_z1}\n")
            except Exception as e:
                print(e)
                pass

    def reset_variables(self):
        self.TOW = self.iTOW = self.fTOW = None
        for svId, satellite in self.satellites.items():
            self.satellites[svId].rawx = None
            self.satellites[svId].sat = None
