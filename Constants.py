from datetime import datetime, timedelta

ApproximateEarthRadius = 6400
GPSAltitude = 20200

OmegaEarthDot = 7.2921151467 * 10e-5
### ПЗ-90
az = 6378137
alpha = 1 / 298.25784
bz = az - alpha * az
### WGS-84
a = 6378136
alpha = 1 / 298.257223563
StartDate = datetime(1980, 1, 1) + timedelta(weeks=2048 + 256)

LLA = [55.690555555555555, 37.858333333333334, 140]
# ECEF = (2842100.6406425796, 4164888.83911829, 3893127.006272498)  # 2800, 4200, 3900 км
