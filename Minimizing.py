import warnings
from datetime import datetime

import numpy as np
from scipy.optimize import minimize, least_squares, differential_evolution, curve_fit
from numpy.linalg import norm
import Constants
from numba import jit

import Transformations


def apply_func(satellites, func):
    def result_function(xyzt):
        return sum(
            [
                func(*xyzt, *params)
                for params in satellites
            ]
        )

    return result_function

def apply_list_func(satellites, func):
    def result_function(xyzt):
        return np.array(
            [
                func(*xyzt, *params)
                for params in satellites
            ]
        )

    return result_function


# @jit(nopython=True)
def func(x, y, z, cdt, xi, yi, zi, rhoi):
    # c = Constants.c
    res = (cdt - rhoi + np.sqrt((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2)) ** 2
    return res

def jac(x, y, z, cdt, xi, yi, zi, rhoi):
    rho = np.sqrt((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2)
    drho = cdt - rhoi + rho
    return np.array([
        2*(x-xi) * drho / rho,
        2*(y-yi) * drho / rho,
        2*(z-zi) * drho / rho,
        2 * drho,
    ])

def constraint(xyzt):
    x, y, z, cdt = xyzt
    # r = np.linalg.norm(np.array([x, y, z]))
    r = Transformations.ecef2lla(x, y, z)[2]
    H = 500
    return H - abs(r-Constants.ae_glonass)

def solve_navigation_task_SLSQP(satellites, bounds=None, Y0=None):
    L = 50e6
    if bounds is None:
        bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]
    # t1 = datetime.now(tz=Constants.tz_utc)
    # for i in range(3):
    #     T = Constants.ECEF[i]
    #     h=0.7e3
    #     bounds[i] = (T-h, T+h)
    # Y0 = np.array(list(Constants.ECEF) + [0])
    con = {'type': 'ineq', 'fun': constraint}
    result = minimize(
        # get_minimize_function(satellites),
        apply_func(satellites, func),
        np.array(Y0) if Y0 is not None else np.array([Constants.ae_glonass, 0, 0, 0]),
        method='SLSQP',
        bounds=bounds,
        constraints=[con],
        jac=apply_func(satellites, jac),
        # jac=get_minimize_derivative(satellites),
        # jac=apply_func(satellites, jac),
        options={'disp': True, 'maxiter': 10000},
        tol=1e-10,
    )
    # t2 = datetime.now(tz=Constants.tz_utc)
    # print((t2-t1).total_seconds())
    print('sqp', Transformations.ecef2lla(*result.x[:-1]))
    print('SQP deiation', constraint(result.x))
    return result

def solve_navigation_task_CF(satellites):

    x_data = []
    y_data = []
    for sat in satellites:
        x_data.append(sat[:-1])
        y_data.append(sat[-1])

    def func(xdata, x, y, z, cdt):
        xi, yi, zi = xdata[:, 0], xdata[:, 1], xdata[:, 2]
        return np.sqrt((xi - x) ** 2 + (yi - y) ** 2 + (zi - z) ** 2) + cdt

    # print('curve fit')
    x, covar = curve_fit(func, x_data, y_data)
    class res:
        def __init__(self, x):
            self.x = x
            self.fun = apply_func(satellites, func)(x)
            self.success = True
    return res(x)



def solve_navigation_task_TC(satellites, Y0=None):
    L = 50e6
    bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]


    con = {'type': 'ineq', 'fun': constraint}
    result = minimize(
        apply_func(satellites, func),
        np.array(Y0) if Y0 else np.array([Constants.ae_glonass, 0, 0, 0]),
        method='trust-constr',
        bounds=bounds,
        # constraints=[con],
        tol=1e-15,
        jac=apply_func(satellites, jac),
    )
    print('tc', Transformations.ecef2lla(*result.x[:-1]))
    print('TC deiation', constraint(result.x))
    return result




def solve_navigation_task_COBYLA(satellites):
    L = 50e6
    bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]

    con = {'type': 'ineq', 'fun': constraint}
    result = minimize(
        apply_func(satellites, func),
        np.array([Constants.ae_glonass, 0, 0, 0]),
        method='COBYLA',
        bounds=bounds,
        constraints=[con],
        tol=1e-10,
        # jac=apply_func(satellites, jac),
    )
    return result




## List Squares


def solve_navigation_task_TRF(satellites):

    # @jit(nopython=True)
    def residuals(params, xi, yi, zi, rhoi):
        x, y, z, cdt = params
        res = cdt - rhoi + np.sqrt((x - xi)**2 + (y - yi)**2 + (z - zi)**2)
        dRho = abs(np.sqrt((x) ** 2 + (y) ** 2 + (z) ** 2)-Constants.ae_glonass)
        # x1, y1, z1 = Constants.ECEF
        # k = abs(np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)) ** 0.003
        # k = abs(np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)) ** 0.003
        H = 500
        return res #* (1 if dRho < H else H)
        # return abs(res)
        # return res**2

    def jac(params, xi, yi, zi, rhoi):
        x, y, z, cdt = params
        rho = np.sqrt((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2)
        J = np.zeros((len(xi), 4))
        J[:, 0] = (x - xi) / rho
        J[:, 1] = (y - yi) / rho
        J[:, 2] = (z - zi) / rho
        J[:, 3] = 1
        return J

    xi, yi, zi, rhoi = [], [], [], []
    for params in satellites:
        xi.append(params[0])
        yi.append(params[1])
        zi.append(params[2])
        rhoi.append(params[3])

    initial_params = np.array([0.0, 0.0, 0.0, 0.0])
    # initial_params = np.array(list(Constants.ECEF) + [-0.008*Constants.c])
    # initial_params = np.array([2e6, 2e6, 4e6, 0])

    # L = 50e6
    # bounds = [[None, None, None, -L], [None, None, None, L]]
    # # bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]
    # # t1 = datetime.now(tz=Constants.tz_utc)
    # for i in range(3):
    #     T = Constants.ECEF[i]
    #     h = 0.7e3
    #     bounds[0][i] = T - h
    #     bounds[1][i] = T + h
    # Y0 = np.array(list(Constants.ECEF) + [0])
    # initial_params  = Y0


    result = least_squares(residuals,
                           initial_params,
                           # x0=initial_params,
                           args=(np.array(xi), np.array(yi), np.array(zi), np.array(rhoi)),
                           # args=(np.array([0])),
                           # bounds=bounds,
                           method='trf',
                           jac=jac,
                           # method='lm',
                           xtol=1e-10,
                           ftol=1e-10,
                           )
    # print(result)
    # print(f'LLA: {Transformations.ecef2lla(*result.x[:-1])}')
    print('trf', Transformations.ecef2lla(*result.x[:-1]))

    return result


def solve_navigation_task_DogBox(satellites):
    def jac(params, xi, yi, zi, rhoi):
        x, y, z, cdt = params
        rho = np.sqrt((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2)
        J = np.zeros((len(xi), 4))
        J[:, 0] = (x - xi) / rho
        J[:, 1] = (y - yi) / rho
        J[:, 2] = (z - zi) / rho
        J[:, 3] = 1
        return J

    # @jit(nopython=True)
    def residuals(params, xi, yi, zi, rhoi):
        x, y, z, cdt = params
        res = cdt - rhoi + np.sqrt((x - xi)**2 + (y - yi)**2 + (z - zi)**2)
        dRho = abs(np.sqrt((x) ** 2 + (y) ** 2 + (z) ** 2)-Constants.ae_glonass)
        # x1, y1, z1 = Constants.ECEF
        # k = abs(np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)) ** 0.003
        # k = abs(np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)) ** 0.003
        H = 500
        return res #* (1 if dRho < H else H)
        # return abs(res)
        # return res**2

    xi, yi, zi, rhoi = [], [], [], []
    for params in satellites:
        xi.append(params[0])
        yi.append(params[1])
        zi.append(params[2])
        rhoi.append(params[3])

    initial_params = np.array([0.0, 0.0, 0.0, -0.005])
    # initial_params = np.array(list(Constants.ECEF) + [-0.008*Constants.c])
    # initial_params = np.array([2e6, 2e6, 4e6, 0])
    # bounds = ([None, None, None, -0.0053], [None, None, None, -0.0048])


    # bounds = ([None, None, None, None], [None, None, None, None])

    result = least_squares(residuals,
                           initial_params,
                           # x0=initial_params,
                           args=(np.array(xi), np.array(yi), np.array(zi), np.array(rhoi)),
                           # args=(np.array([0])),
                           # bounds=bounds,
                           method='dogbox',
                           jac=jac,
                           # method='lm',
                           xtol=1e-10,
                           ftol=1e-10,
                           )
    # print(result)
    # print(f'LLA: {Transformations.ecef2lla(*result.x[:-1])}')
    print('db', Transformations.ecef2lla(*result.x[:-1]))

    return result

def solve_navigation_task_LevMar(satellites):
    def jac(params, xi, yi, zi, rhoi):
        x, y, z, cdt = params
        rho = np.sqrt((x - xi)**2 + (y - yi)**2 + (z - zi)**2)
        J = np.zeros((len(xi), 4))
        J[:, 0] = (x-xi)/rho
        J[:, 1] = (y-yi)/rho
        J[:, 2] = (z-zi)/rho
        J[:, 3] = 1
        return J

    # @jit(nopython=True)
    def residuals(params, xi, yi, zi, rhoi):
        x, y, z, cdt = params
        res = cdt - rhoi + np.sqrt((x - xi)**2 + (y - yi)**2 + (z - zi)**2)
        dRho = abs(np.sqrt((x) ** 2 + (y) ** 2 + (z) ** 2)-Constants.ae_glonass)
        # x1, y1, z1 = Constants.ECEF
        # k = abs(np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)) ** 0.003
        # k = abs(np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)) ** 0.003
        H = 500
        return res #* (1 if dRho < H else H)
        # return abs(res)
        # return res**2

    xi, yi, zi, rhoi = [], [], [], []
    for params in satellites:
        xi.append(params[0])
        yi.append(params[1])
        zi.append(params[2])
        rhoi.append(params[3])

    initial_params = np.array([0.0, 0.0, 0.0, -0.005])
    # initial_params = np.array(list(Constants.ECEF) + [-0.008*Constants.c])
    # initial_params = np.array([2e6, 2e6, 4e6, 0])
    #

    result = least_squares(residuals,
                           initial_params,
                           # x0=initial_params,
                           args=(np.array(xi), np.array(yi), np.array(zi), np.array(rhoi)),
                           # args=(np.array([0])),
                           method='lm',
                           jac=jac,
                           # method='lm',
                           xtol=1e-10,
                           ftol=1e-10,
                           )
    # print(result)
    # print(f'LLA: {Transformations.ecef2lla(*result.x[:-1])}')
    print('LM', Transformations.ecef2lla(*result.x[:-1]))

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




if __name__ == '__main__':
    import pandas as pd
    df = pd.read_csv('data.csv', header=0, index_col=0)
    sats =i
    # for i,

