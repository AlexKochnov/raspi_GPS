import numpy as np
from scipy.optimize import minimize

import Constants

TimeCoefficient = 1e0  # 1 -> 1 sec, 1e3 -> 1 ms, 1e6 -> 1 mk s


def calc_rho(satellite, xyzt):
    result = np.sqrt(
        (satellite.eph_coord[0] - xyzt[0]) ** 2
        + (satellite.eph_coord[1] - xyzt[1]) ** 2
        + (satellite.eph_coord[2] - xyzt[2]) ** 2
    )
    # result = np.linalg.norm(satellite.eph_coord - xyzt[:-1])
    # print(result)
    return result


def calc_dRho(satellite, xyzt):
    return calc_rho(satellite, xyzt) + Constants.c * (xyzt[3] / TimeCoefficient) - satellite.rawx.prMes  # + xyzt[4]


def get_minimize_function(good_eph):
    def minimize_function(xyzt):
        arr = list(
            [calc_dRho(satellite, xyzt) ** 2 for satellite in good_eph])
        # print(arr)
        # print(sum(arr))
        return sum(arr)

    return minimize_function


def get_minimize_derivative(good_eph):
    def minimize_derivative(xyzt):
        X = list([
            -2 * (satellite.eph_coord[0] - xyzt[0])
            * calc_dRho(satellite, xyzt)
            / calc_rho(satellite, xyzt)
            for satellite in good_eph])
        Y = list([
            -2 * (satellite.eph_coord[1] - xyzt[1])
            * calc_dRho(satellite, xyzt)
            / calc_rho(satellite, xyzt)
            for satellite in good_eph])
        Z = list([
            -2 * (satellite.eph_coord[2] - xyzt[2])
            * calc_dRho(satellite, xyzt)
            / calc_rho(satellite, xyzt)
            for satellite in good_eph])
        T = list([2 * Constants.c * (calc_dRho(satellite, xyzt) / TimeCoefficient) for satellite in good_eph])
        # Q = list([2 * dRho(satellite, xyzt) for satellite in good_eph])
        return sum(X), sum(Y), sum(Z), sum(T)

    return minimize_derivative


def solve_navigation_task(satellites):
    return minimize(
        get_minimize_function(satellites),
        np.array([0, 0, 0, 0]),
        method='BFGS',
        jac=get_minimize_derivative(satellites),
        options={'disp': True, 'maxiter': 100},
        tol=1e-10,
    )
