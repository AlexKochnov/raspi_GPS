from math import *
def ecef2lla(x, y, z):
    # x, y and z are scalars or vectors in meters

    a = 6378137
    a_sq = a ** 2
    e = 8.181919084261345e-2
    e_sq = 6.69437999014e-3

    f = 1 / 298.257223563
    b = a * (1 - f)

    # calculations:
    r = sqrt(x ** 2 + y ** 2)
    ep_sq = (a ** 2 - b ** 2) / b ** 2
    ee = (a ** 2 - b ** 2)
    f = (54 * b ** 2) * (z ** 2)
    g = r ** 2 + (1 - e_sq) * (z ** 2) - e_sq * ee * 2
    c = (e_sq ** 2) * f * r ** 2 / (g ** 3)
    s = (1 + c + sqrt(c ** 2 + 2 * c)) ** (1 / 3.)
    p = f / (3. * (g ** 2) * (s + (1. / s) + 1) ** 2)
    q = sqrt(1 + 2 * p * e_sq ** 2)
    r_0 = -(p * e_sq * r) / (1 + q) + sqrt(
        0.5 * (a ** 2) * (1 + (1. / q)) - p * (z ** 2) * (1 - e_sq) / (q * (1 + q)) - 0.5 * p * (r ** 2))
    u = sqrt((r - e_sq * r_0) ** 2 + z ** 2)
    v = sqrt((r - e_sq * r_0) ** 2 + (1 - e_sq) * z ** 2)
    z_0 = (b ** 2) * z / (a * v)
    h = u * (1 - b ** 2 / (a * v))
    phi = atan((z + ep_sq * z_0) / r)
    lambd = atan2(y, x)

    return phi * 180 / pi, lambd * 180 / pi, h