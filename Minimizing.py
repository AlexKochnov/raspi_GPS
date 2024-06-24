import warnings

import numpy as np
from scipy.optimize import minimize, least_squares
from numpy.linalg import norm
import Constants

TimeCoefficient = 1e1  # 1 -> 1 sec, 1e3 -> 1 ms, 1e6 -> 1 mk s

# warnings.simplefilter('error', RuntimeWarning)


def calc_rho(satellite, xyzt):
    # with warnings.catch_warnings(record=True) as w:
    #     warnings.simplefilter("always")
    # try:
    #     if norm(xyzt) > 1:
    #         a = 0
    result = np.sqrt(
        (satellite.eph_coord[0] - xyzt[0]) ** 2
        + (satellite.eph_coord[1] - xyzt[1]) ** 2
        + (satellite.eph_coord[2] - xyzt[2]) ** 2
    )
    # except RuntimeWarning as e:
    #     a = 0
    # result = np.linalg.norm(satellite.eph_coord - xyzt[:-1])
    # print(result)
    return result


def calc_dRho(satellite, xyzt):
    return calc_rho(satellite, xyzt) + Constants.c * (xyzt[3] / TimeCoefficient) - satellite.rawx.prMes  # + xyzt[4]


def get_minimize_function(good_eph):
    def minimize_function(xyzt):
        # if norm(np.array(xyzt)) > 1e15:
        #     return 1e20
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
        return np.array([sum(X), sum(Y), sum(Z), sum(T)])

    return minimize_derivative


def solve_navigation_task1(satellites):
    return minimize(
        get_minimize_function(satellites),
        np.array([0, 0, 0, 0]),
        method='BFGS',
        jac=get_minimize_derivative(satellites),
        options={'disp': True, 'maxiter': 100},
        tol=1e-10,
    )


def my_minimize(satellites, lin_d=1):
    return GradientDescent(
        func=get_minimize_function(satellites),
        jac=get_minimize_derivative(satellites),
        x0=np.array([0, 0, 0, 0]),
        eps=1e-10,
        lin_d=lin_d
    )


def OrderSearch(func, x0, d, eps, disp=False):
    while abs(d) > eps:
        # print(d, x0, func(x0))
        x1 = x0 + d
        if func(x0) < func(x1):
            d = -d / 4
        x0 = x1
    return x1


def GradientDescent(func, jac, x0, eps, lin_d=1):
    jac0 = np.array([1] * len(x0))
    p0 = np.array([0] * len(x0))
    x1 = x0
    k = 0
    while norm(jac0) > eps:
        # print(k, x0, jac0)
        k += 1
        jac1 = jac(x1)
        # b0 = norm(jac1)**2 / norm(jac0)**2
        # b0 = min(b0, 1e8)
        # p1 = jac1 + b0 * p0
        p1 = jac1  # / norm(jac1)

        lin_func = lambda L: func(x1 - L * p1)
        lmb = OrderSearch(lin_func, 0, 1000, eps)
        # lmb = 1e-6

        x0 = x1
        x1 = x0 - lmb * p1
        jac0 = jac1
        p0 = p1
    return x1


def apply_func(satellites, func):
    def result_function(xyzt):
        return sum(
            [
                func(*xyzt, *sat.eph_coord, sat.rawx.prMes)
                for sat in satellites
            ]
        )

    return result_function


def func(x, y, z, cdt, xi, yi, zi, rhoi):
    # c = Constants.c
    res = (cdt - rhoi + np.sqrt((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2)) ** 2
    return res


def jac(x, y, z, cdt, xi, yi, zi, rhoi):
    c = Constants.c
    res = \
        [((2 * x - 2 * xi) * (cdt - rhoi + ((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2) ** (1 / 2))) / (
                (x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2) ** (1 / 2),
         ((2 * y - 2 * yi) * (cdt - rhoi + ((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2) ** (1 / 2))) / (
                 (x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2) ** (1 / 2),
         ((2 * z - 2 * zi) * (cdt - rhoi + ((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2) ** (1 / 2))) / (
                 (x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2) ** (1 / 2),
         2 * (cdt - rhoi + ((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2) ** (1 / 2))]
    return np.array(res)


def hess(x, y, z, cdt, xi, yi, zi, rhoi):
    # c = Constants.c

    RHO2 = (x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2
    DRHO = (cdt - rhoi + RHO2 ** 0.5)
    D2X = (2 * x - 2 * xi)
    D2Y = (2 * y - 2 * yi)
    D2Z = (2 * z - 2 * zi)

    res = \
        [
            [
                (2 * DRHO) / RHO2 ** (1 / 2) + D2X ** 2 / (2 * RHO2) - (D2X ** 2 * DRHO) / (2 * RHO2 ** (3 / 2)),
                (D2X * D2Y) / (2 * RHO2) - (D2X * D2Y * DRHO) / (2 * RHO2 ** (3 / 2)),
                (D2X * D2Z) / (2 * RHO2) - (D2X * D2Z * DRHO) / (2 * RHO2 ** (3 / 2)),
                (D2X) / RHO2 ** (1 / 2),
            ],
            [
                (D2X * D2Y) / (2 * RHO2) - (D2X * D2Y * DRHO) / (2 * RHO2 ** (3 / 2)),
                (2 * DRHO) / RHO2 ** (1 / 2) + D2Y ** 2 / (2 * RHO2) - (D2Y ** 2 * DRHO) / (2 * RHO2 ** (3 / 2)),
                (D2Y * D2Z) / (2 * RHO2) - (D2Y * D2Z * DRHO) / (2 * RHO2 ** (3 / 2)),
                (D2Y) / RHO2 ** (1 / 2),
            ],
            [
                (D2X * D2Z) / (2 * RHO2) - (D2X * D2Z * DRHO) / (2 * RHO2 ** (3 / 2)),
                (D2Y * D2Z) / (2 * RHO2) - (D2Y * D2Z * DRHO) / (2 * RHO2 ** (3 / 2)),
                (2 * DRHO) / RHO2 ** (1 / 2) + D2Z ** 2 / (2 * RHO2) - (D2Z ** 2 * DRHO) / (2 * RHO2 ** (3 / 2)),
                (D2Z) / RHO2 ** (1 / 2)
            ],
            [
                (D2X) / RHO2 ** (1 / 2),
                (D2Y) / RHO2 ** (1 / 2),
                (D2Z) / RHO2 ** (1 / 2),
                2,
            ]
        ]

    return np.array(res)

# def inv_hess(x, y, z, t, xi, yi, zi, rhoi):
#     c = Constants.c
#
#     RHO2 = (x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2
#     DRHO = (c * t - rhoi + RHO2 ** 0.5)
#     D2X = (2 * x - 2 * xi)
#     D2Y = (2 * y - 2 * yi)
#     D2Z = (2 * z - 2 * zi)
#     try:
#         res = \
#             [
#                 [(RHO2 ** (1 / 2) * (D2Y ** 2 + D2Z ** 2 - 4 * RHO2)) / (
#                             2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)),
#                  -(D2X * D2Y * RHO2 ** (1 / 2)) / (
#                              2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)),
#                  -(D2X * D2Z * RHO2 ** (1 / 2)) / (
#                              2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)),
#                  (D2X * RHO2) / (DRHO * c * D2X ** 2 + DRHO * c * D2Y ** 2 + DRHO * c * D2Z ** 2 - 4 * DRHO * RHO2 * c)],
#                 [-(D2X * D2Y * RHO2 ** (1 / 2)) / (
#                             2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)), (
#                              RHO2 ** (1 / 2) * (D2X ** 2 + D2Z ** 2 - 4 * RHO2)) / (
#                              2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)), -(
#                             D2Y * D2Z * RHO2 ** (1 / 2)) / (
#                              2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)), (D2Y * RHO2) / (
#                              DRHO * c * D2X ** 2 + DRHO * c * D2Y ** 2 + DRHO * c * D2Z ** 2 - 4 * DRHO * RHO2 * c)],
#                 [-(D2X * D2Z * RHO2 ** (1 / 2)) / (
#                             2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)), -(
#                             D2Y * D2Z * RHO2 ** (1 / 2)) / (
#                              2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)), (
#                              RHO2 ** (1 / 2) * (D2X ** 2 + D2Y ** 2 - 4 * RHO2)) / (
#                              2 * (DRHO * D2X ** 2 + DRHO * D2Y ** 2 + DRHO * D2Z ** 2 - 4 * DRHO * RHO2)), (D2Z * RHO2) / (
#                              DRHO * c * D2X ** 2 + DRHO * c * D2Y ** 2 + DRHO * c * D2Z ** 2 - 4 * DRHO * RHO2 * c)],
#                 [(D2X * RHO2) / (DRHO * c * D2X ** 2 + DRHO * c * D2Y ** 2 + DRHO * c * D2Z ** 2 - 4 * DRHO * RHO2 * c), (
#                             D2Y * RHO2) / (
#                              DRHO * c * D2X ** 2 + DRHO * c * D2Y ** 2 + DRHO * c * D2Z ** 2 - 4 * DRHO * RHO2 * c), (
#                              D2Z * RHO2) / (
#                              DRHO * c * D2X ** 2 + DRHO * c * D2Y ** 2 + DRHO * c * D2Z ** 2 - 4 * DRHO * RHO2 * c), -(
#                             D2X ** 2 * RHO2 ** (
#                                 1 / 2) - D2Y ** 2 * DRHO - D2Z ** 2 * DRHO - D2X ** 2 * DRHO + D2Y ** 2 * RHO2 ** (
#                                         1 / 2) + D2Z ** 2 * RHO2 ** (1 / 2) + 4 * DRHO * RHO2) / (2 * c * (
#                             DRHO * c * D2X ** 2 + DRHO * c * D2Y ** 2 + DRHO * c * D2Z ** 2 - 4 * DRHO * RHO2 * c))]
#             ]
#     except Exception as e:
#         res = [0, 0, 0, 0]
#         print(e)
#     return res

def Newton(func, jac, hess, x0, jtol, xtol):
    k = 0
    dx = None
    while k < 30:
        dx = np.linalg.inv(hess(x0)) @ jac(x0)
        # dx = inv_hess(x0) @ jac(x0)
        x0 = x0 - dx
        k += 1
        if norm(jac(x0)) < jtol or norm(dx) < xtol:
            break
    return x0, k, norm(dx)


def solve_navigation_task(satellites) -> list[float]:
    solve, iter, xtol = Newton(
        func=apply_func(satellites, func),
        jac=apply_func(satellites, jac),
        hess=apply_func(satellites, hess),
        # inv_hess=apply_func(satellites, inv_hess),
        x0=np.array([0, 0, 0, 0]),
        jtol=0,
        xtol=1e-7
    )
    return solve


def solve_navigation_task_SLSQP(satellites, bounds=[]):
    return minimize(
        # get_minimize_function(satellites),
        apply_func(satellites, func),
        np.array([0, 0, 0, 0]),
        method='SLSQP',
        bounds=bounds,
        # jac=get_minimize_derivative(satellites),
        # jac=apply_func(satellites, jac),
        options={'disp': True, 'maxiter': 100},
        tol=1e-10,
    )


def solve_navigation_task_LevMar(satellites):

    def residuals(params, xi, yi, zi, rhoi):
        x, y, z, cdt = params
        res = cdt - rhoi + np.sqrt((x - xi)**2 + (y - yi)**2 + (z - zi)**2)
        # res = 0
        # for sat in satellites:
        #     res += norm(cdt - sat.rawx.prMes + np.sqrt((x - sat.eph_coord[0])**2 + (y - sat.eph_coord[1])**2 + (z - sat.eph_coord[2])**2))
        return res

    xi, yi, zi, rhoi = [], [], [], []
    for sat in satellites:
        xi.append(sat.eph_coord[0])
        yi.append(sat.eph_coord[1])
        zi.append(sat.eph_coord[2])
        rhoi.append(sat.rawx.prMes)

    initial_params = np.array([0.0, 0.0, 0.0, 0.0])
    # initial_params = np.array([2e6, 2e6, 4e6, 0])

    # bounds = ([None, None, None, None], [None, None, None, None])

    result = least_squares(residuals,
                           initial_params,
                           # x0=initial_params,
                           args=(np.array(xi), np.array(yi), np.array(zi), np.array(rhoi)),
                           # args=(np.array([0])),
                           # bounds=bounds,
                           method='lm',
                           xtol=1e-8
                           )

    return result
