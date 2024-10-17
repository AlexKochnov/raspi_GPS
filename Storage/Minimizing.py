import numpy as np
from scipy.optimize import minimize, least_squares, differential_evolution

from Utils import Constants, Transformations

def residuals_df(params, satellites):
    res = params[-1] - satellites.T[:, -1] + np.linalg.norm(satellites.T[:, :-1] - params[:-1], axis=1)
    return res

def jac_df(params, satellites):
    xi, yi, zi, pr = satellites
    x, y, z, cdt = params
    rho = np.sqrt((x - xi) ** 2 + (y - yi) ** 2 + (z - zi) ** 2)
    J = np.zeros((len(xi), 4))
    J[:, 0] = (x - xi) / rho
    J[:, 1] = (y - yi) / rho
    J[:, 2] = (z - zi) / rho
    J[:, 3] = 1
    return J

def constraint(xyzt):
    x, y, z, cdt = xyzt
    r = Transformations.ecef2lla(x, y, z)[2]
    H = 5000
    return H - abs(r)

def get_func(satellites):
    return lambda xyzt: sum([x**2 for x in residuals_df(xyzt, satellites)] )


def get_jac(satellites):
    return lambda xyzt: np.array([sum([y**2 for y in x])  for x in jac_df(xyzt, satellites).T])

def solve_navigation_task_SLSQP(satellites, bounds=None, Y0=None):
    L = 50e6
    if bounds is None:
        bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]
    cdt = np.linalg.norm(satellites[:3, 0] - np.array(Constants.ECEF)) - satellites[-1, 0]
    Y0 = np.array(list(Constants.ECEF) + [-cdt])
    Y0 -= 200
    # Y0 = np.array([Constants.a, 0, 0, 0])
    con = {'type': 'ineq', 'fun': constraint}
    result = minimize(
        get_func(satellites),
        np.array(Y0) if Y0 is not None else np.array([Constants.ae_glonass, 0, 0, 0]),
        method='SLSQP',
        bounds=bounds,
        # constraints=[con],
        jac=get_jac(satellites),
        options={'disp': True, 'maxiter': 500},
        tol=1e-5,
    )
    print('constraint', constraint(result.x),)
    return result

def solve_navigation_task_TC(satellites, Y0=None):
    L = 50e6
    bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]

    con = {'type': 'ineq', 'fun': constraint}
    N = len(satellites.T)
    # cdt = sum([np.linalg.norm(satellites[:3, i] - np.array(Constants.ECEF)) - satellites[-1, i] for i in range(N)])/N
    # Y0 = np.array(list(Constants.ECEF) + [-cdt])
    # Y0 = np.array([Constants.a, 0, 0, 0])
    # Y0 -= 1e2
    # print(Y0, Y0[-1]/Constants.c)
    result = minimize(
        get_func(satellites),
        np.array(Y0) if Y0 is not None else np.array([Constants.ae_glonass, 0, 0, 0]),
        # np.array(Y0) if Y0 is not None else np.array([0, 0, 0, 0]),
        method='trust-constr',
        bounds=bounds,
        # constraints=[con],
        tol=1e-2,
        jac=get_jac(satellites),
        options={'disp': True, 'maxiter': 500},
    )
    for i in range(len(satellites.T)):
        print((np.linalg.norm(satellites[:3, i] - np.array(Constants.ECEF)) - satellites[-1, i]) / Constants.c + result.x[-1] / Constants.c)

    print('constraint', constraint(result.x), Transformations.ecef2lla(*result.x[:-1])[2])
    return result

## List Squares

def solve_navigation_task_TRF(satellites):
    return solve_least_squares(satellites, 'trf')

def solve_navigation_task_DogBox(satellites):
    return solve_least_squares(satellites, 'dogbox')

def solve_navigation_task_LevMar(satellites):
    return solve_least_squares(satellites, 'lm')

def solve_least_squares(satellites, method):
    initial_params = np.array([0.0, 0.0, 0.0, 0.0])
    N = len(satellites.T)
    # cdt = sum([np.linalg.norm(satellites[:3, i] - np.array(Constants.ECEF)) - satellites[-1, i] for i in range(N)]) / N
    # initial_params = np.array(list(Constants.ECEF) + [-cdt])
    # print(initial_params)
    result = least_squares(
        residuals_df,
        initial_params,
        args=(satellites,),
        method=method,
        jac=jac_df,
        xtol=1e-10,
        ftol=1e-10,
    )
    return result

def solve_navigation_task_GO(satellites, bounds=None):
    L = 50e6
    if bounds is None:
        bounds = [(-L, L), (-L, L), (-L, L), (-L, L)]
    return differential_evolution(
        get_func(satellites),
        bounds,
        tol=1e-10,
    )
