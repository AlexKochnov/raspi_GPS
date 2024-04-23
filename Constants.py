from datetime import datetime, timedelta
from pytz import timezone

tz_moscow = timezone('Europe/Moscow')
tz_utc = timezone('utc')

GPSAltitude = 20200

ApproximateEarthRadius = 6400

OmegaEarthDot = 7.2921151467 * 10e-5
mu = 3.9860044 * 1e14

c = 299792458

### ПЗ-90
az = 6378137
alpha = 1 / 298.25784
bz = az - alpha * az

### WGS-84
a = 6378136
alpha = 1 / 298.257223563
b = a - alpha * a


# Moscow:
LLA = [55.690555555555555, 37.858333333333334, 140]
# ECEF = (2842100.6406425796, 4164888.83911829, 3893127.006272498)  # 2800, 4200, 3900 км
import pymap3d as pm
ECEF = pm.geodetic2ecef(*LLA)
ECI = pm.geodetic2ecef(*LLA)


