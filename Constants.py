import math
from datetime import datetime, timedelta
from pytz import timezone

import Transformations
from Settings import *


def DT2TOW(timestamp: datetime, step=1):
    # timestamp = datetime(2024, 7, 12, 14, 18, 4, 956055, tzinfo=Constants.tz_moscow)
    dt = (timestamp - gps_epoch).total_seconds() + leapS
    TOW = dt % week_seconds
    week = int(dt // week_seconds)
    TOW = math.floor(TOW / step) * step
    return TOW, week


def get_now_TOW():
    return DT2TOW(datetime.now(tz=tz_utc), STEP)


tz_moscow = timezone('Europe/Moscow')
tz_utc = timezone('utc')
gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=tz_utc)

week_seconds = 7 * 24 * 3600
leapS = 18
STEP = 1
BASE_TIME_STAMP = get_now_TOW

GPSAltitude = 20200

ApproximateEarthRadius = 6400

OmegaEarthDot = 7.2921151467 * 1e-5
mu = 3.9860044 * 1e14

c = 299792458

## ПЗ-90
az = 6378137
alpha = 1 / 298.25784
bz = az - alpha * az

## WGS-84
a = 6378136
alpha = 1 / 298.257223563
b = a - alpha * a

# TODO: delete
# Moscow:
LLA = [55.690555555555555, 37.858333333333334, 140]
# ECEF = (2842100.6406425796, 4164888.83911829, 3893127.006272498)  # 2800, 4200, 3900 км
# import pymap3d as pm
# ECEF = pm.geodetic2ecef(*LLA)
ECEF = Transformations.lla2ecef(*LLA)




# def DT2TOW(self, timestamp: datetime, step=1):
#     # timestamp = datetime(2024, 7, 12, 14, 18, 4, 956055, tzinfo=Constants.tz_moscow)
#     dt = (timestamp - Constants.gps_epoch).total_seconds() + Constants.leapS
#     TOW = dt % Constants.week_seconds
#     week = int(dt // Constants.week_seconds)
#     TOW = math.floor(TOW/step) * step
#     return TOW, week