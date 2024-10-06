import numpy as np


class RK45Solver:
    order = 5
    error_estimator_order = 4
    n_stages = 6
    C = np.array([0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1])
    A = np.array([
        [0, 0, 0, 0, 0],
        [1 / 5, 0, 0, 0, 0],
        [3 / 40, 9 / 40, 0, 0, 0],
        [44 / 45, -56 / 15, 32 / 9, 0, 0],
        [19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729, 0],
        [9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656]
    ])
    B = np.array([35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84])
    E = np.array([-71 / 57600, 0, 71 / 16695, -71 / 1920, 17253 / 339200, -22 / 525,
                  1 / 40])
    P = np.array([
        [1, -8048581381 / 2820520608, 8663915743 / 2820520608,
         -12715105075 / 11282082432],
        [0, 0, 0, 0],
        [0, 131558114200 / 32700410799, -68118460800 / 10900136933,
         87487479700 / 32700410799],
        [0, -1754552775 / 470086768, 14199869525 / 1410260304,
         -10690763975 / 1880347072],
        [0, 127303824393 / 49829197408, -318862633887 / 49829197408,
         701980252875 / 199316789632],
        [0, -282668133 / 205662961, 2019193451 / 616988883, -1453857185 / 822651844],
        [0, 40617522 / 29380423, -110615467 / 29380423, 69997945 / 29380423]])

    def __init__(self, fun, y0, t0, step=1):
        self.fun = fun
        self.t = [t0]
        self.y = [y0]
        self.f = fun(t0, y0)
        self.step = step
        self.n = y0.size
        self.K = np.empty((self.n_stages + 1, self.n), dtype=y0.dtype)

    def get(self, t):
        if t < self.t[0]:
            return None
        while t > self.t[-1]:
            self.next()
        return self.y[-1]

    def next(self):
        self.t.append(self.t[-1] + self.step)
        y, self.f = self.rk_step(self.fun, self.t[-1], self.y[-1], self.f, self.step, self.A, self.B, self.C, self.K)
        self.y.append(y)
        return self.t[-1], self.y[-1]

    @staticmethod
    def rk_step(fun, t, y, f, h, A, B, C, K):
        K[0] = f
        for s, (a, c) in enumerate(zip(A[1:], C[1:]), start=1):
            dy = np.dot(K[:s].T, a[:s]) * h
            K[s] = fun(t + c * h, y + dy)
        y_new = y + h * np.dot(K[:-1].T, B)
        f_new = fun(t + h, y_new)
        K[-1] = f_new
        return y_new, f_new
