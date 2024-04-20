import numpy as np
from scipy.optimize import minimize

import Constants


def calc_rho(satellite, xyzt):
    result = np.sqrt(
        (satellite.eph_coord[0] - xyzt[0]) ** 2
        + (satellite.eph_coord[1] - xyzt[1]) ** 2
        + (satellite.eph_coord[2] - xyzt[2]) ** 2
    )
    # print(result)
    return result


def get_minimize_function(good_eph):
    def minimize_function(xyzt):
        arr = list(
            [(calc_rho(satellite, xyzt) + Constants.с * xyzt[3] - satellite.rawx.prMes) ** 2 for satellite in good_eph])
        # print(arr)
        # print(sum(arr))
        return sum(arr)

    return minimize_function


def get_minimize_derivative(good_eph):
    def minimize_derivative(xyzt):
        def dRho(satellite):
            return calc_rho(satellite, xyzt) + Constants.с * xyzt[3] - satellite.rawx.prMes

        X = list([
            -2 * (satellite.eph_coord[0] - xyzt[0])
            * dRho(satellite)
            / calc_rho(satellite, xyzt)
            for satellite in good_eph])
        Y = list([
            -2 * (satellite.eph_coord[1] - xyzt[1])
            * dRho(satellite)
            / calc_rho(satellite, xyzt)
            for satellite in good_eph])
        Z = list([
            -2 * (satellite.eph_coord[2] - xyzt[2])
            * dRho(satellite)
            / calc_rho(satellite, xyzt)
            for satellite in good_eph])
        T = list([2 * Constants.с * dRho(satellite) for satellite in good_eph])
        return sum(X), sum(Y), sum(Z), sum(T)

    return minimize_derivative


def solve_navigation_task(satellites):
    return minimize(
        get_minimize_function(satellites),
        np.array([0, 0, 0, 0]),
        jac=get_minimize_derivative(satellites),
        options={'xtol': 1e-8, 'disp': True, 'maxiter': 100}
    )