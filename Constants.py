from datetime import datetime, timedelta

from pytz import timezone
import pymap3d as pm
# import Transformations

tz_moscow = timezone('Europe/Moscow')
tz_utc = timezone('utc')
gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=tz_utc)
glonass_epoch = datetime(1996, 1, 1, 0, 0, 0, tzinfo=tz_utc)
glonass_4years = timedelta(days=1461)

week_seconds = 7 * 24 * 3600
leapS = 18
STEP = 1

ApproximateGPSAltitude = 20200
ApproximateEarthRadius = 6400

F = -4.442807633 * 1e-10    # = -2sqrt(mu) / c^2 sec/sqrt(m)
OmegaEarthDot = 7.2921151467 * 1e-5
mu = 3.986004418 * 1e14
ae_glonass = 6378136
J20_glonass = 1082.62575e-6

c = 299792458

## ПЗ-90
az = 6378137
alphaz = 1 / 298.25784
bz = az - alphaz * az

## WGS-84
a = 6378136
alpha = 1 / 298.257223563
b = a - alpha * a

# TODO: delete
# Moscow:
LLA = [55.690555555555555, 37.858333333333334, 140] # Дом
# LLA = [55.569861111111116, 38.805027777777774, 140] # Дача
# ECEF = (2842100.6406425796, 4164888.83911829, 3893127.006272498)  # 2800, 4200, 3900 км
# import pymap3d as pm
ECEF = pm.geodetic2ecef(*LLA)
# ECEF = Transformations.lla2ecef(*LLA)
sat_calc_coord_delay = 0



# def DT2TOW(self, timestamp: datetime, step=1):
#     # timestamp = datetime(2024, 7, 12, 14, 18, 4, 956055, tzinfo=Constants.tz_moscow)
#     dt = (timestamp - Constants.gps_epoch).total_seconds() + Constants.leapS
#     TOW = dt % Constants.week_seconds
#     week = int(dt // Constants.week_seconds)
#     TOW = math.floor(TOW/step) * step
#     return TOW, week