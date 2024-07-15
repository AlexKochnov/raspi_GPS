import warnings

import numpy as np
from scipy.optimize import minimize, least_squares, differential_evolution
from numpy.linalg import norm
import Constants


def apply_func(satellites, func):
    def result_function(xyzt):
        return sum(
            [
                func(*xyzt, *params)
                for params in satellites
            ]
        )

    return result_function


def func(x, y, z, cdt, xi, yi, zi, rhoi):
    # c = Constants.c
    res = (cdt - rhoi + np.sqrt((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2)) ** 2
    return res


def solve_navigation_task_SLSQP(satellites, bounds=None):
    if bounds is None:
        bounds = [(None, None), (None, None), (None, None), (None, None)]
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
        # return abs(res)
        return res**2

    xi, yi, zi, rhoi = [], [], [], []
    for params in satellites:
        xi.append(params[0])
        yi.append(params[1])
        zi.append(params[2])
        rhoi.append(params[3])

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

def solve_navigation_task_GO(satellites, bounds=None):
    L = 50e6
    if bounds is None:
        bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]
    return differential_evolution(
        apply_func(satellites, func),
        bounds,
        tol=1e-10,
    )