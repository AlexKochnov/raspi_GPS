from math import sin, cos, radians, sqrt

import numpy as np
import pymap3d as pm

import Constants
from Constants import OmegaEarthDot


def aer2eci(azim, elev, dist, lat, lon, alt, time_sec):
    lon_corrected = lon - time_sec * Constants.OmegaEarthDot * 180 / np.pi
    return pm.aer2ecef(azim, elev, dist, lat, lon_corrected, alt)


def aer2ecef(azim, elev, dist, lat, lon, alt):
    def R(phi, lam):
        phi = np.radians(phi)
        lam = np.radians(lam)
        return np.array([
            [-sin(phi) * cos(lam), -sin(phi) * sin(lam), cos(phi)],
            [-sin(lam), cos(lam), 0],
            [-cos(phi) * cos(lam), -cos(phi) * sin(lam), -sin(phi)],
        ])

    az_rad = radians(azim)
    el_rad = radians(elev)
    E, N, U = [sin(az_rad) * cos(el_rad) * dist, cos(az_rad) * cos(el_rad) * dist, sin(el_rad) * dist]
    NED = np.array([N, E, -U])
    UVW = np.transpose(R(lat, lon)) @ NED
    # XYZ = pm.geodetic2ecef(lat, lon, alt)
    XYZ = lla2ecef(lat, lon, alt)
    ECEF = UVW + XYZ
    return ECEF


def lla2ecef(lat, lon, alt):
    B = np.radians(lat)
    L = np.radians(lon)
    H = alt
    e2 = 2 * Constants.alpha - Constants.alpha ** 2
    N = Constants.a / sqrt(1 - e2 * sin(B) ** 2)
    X = (N + H) * cos(B) * cos(L)
    Y = (N + H) * cos(B) * sin(L)
    Z = ((1 - e2) * N + H) * sin(B)
    return X, Y, Z


def blh2ecef(b, l, h):
    b = np.radians(b)
    l = np.radians(l)
    num = (Constants.az ** 2 * cos(b)) ** 2 + (Constants.bz ** 2 * sin(b)) ** 2
    den = (Constants.az * cos(b)) ** 2 + (Constants.bz * sin(b)) ** 2

    Rz = sqrt(num / den)

    x = (Rz + h) * cos(b) * cos(l)
    y = (Rz + h) * cos(b) * sin(l)
    z = (Rz + h) * sin(b)
    return x, y, z


def rotateZ(x, y, z, ang):
    return (
        x * np.cos(ang) + y * np.sin(ang),
        - x * np.sin(ang) + y * np.cos(ang),
        z
    )


def ecef2eci(t, x, y, z):
    return rotateZ(x, y, z, t * OmegaEarthDot)


def eci2ecef(t, x, y, z):
    return rotateZ(x, y, z, - t * OmegaEarthDot)
