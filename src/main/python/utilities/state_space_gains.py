import numpy as np
import scipy
import scipy.signal


class StateSpaceGains(object):

    def __init__(self, A, B, C, D, Q_noise, R_noise, K, L, dt: float, name: str):
        self.A = np.asmatrix(A)
        self.B = np.asmatrix(B)
        self.C = np.asmatrix(C)
        self.D = np.asmatrix(D)

        self.Q_noise = np.asmatrix(Q_noise)
        self.R_noise = np.asmatrix(R_noise)

        self.K = np.asmatrix(K)
        self.L = np.asmatrix(L)

        self.dt = dt

        self.name = name

        self.is_controllable = self.check_controllability()
        self.is_observable = self.check_observability()

        self.check_system_validity()

    def check_controllability(self) -> bool:

        return True

    def check_observability(self) -> bool:

        n = self.C.shape[0]
        m = self.A.shape[1]
        obsv = np.asmatrix(np.zeros((n*m, m)))
        current_submatrix = self.C

        for i in range(m):
            obsv[i*n:i*n+n, :m] = current_submatrix
            current_submatrix = current_submatrix * self.A

        return np.linalg.matrix_rank(obsv) == self.A.shape[0]

    def check_system_validity(self):
        assert self.A.shape[0] == self.A.shape[1],                                      \
            "A must be square"

        assert self.B.shape[0] == self.A.shape[0],                                      \
            "A and B must have the same number of rows"
    
        assert self.C.shape[1] == self.A.shape[0],                                      \
            "C must have as many columns as there are states"

        assert self.D.shape[0] == self.C.shape[0],                                      \
            "C and D must have the same number of rows"
        assert self.D.shape[1] == self.B.shape[1],                                      \
            "B and D must have the same number of columns"

        assert self.Q_noise.shape[0] == self.Q_noise.shape[1],                          \
            "Q must be square"
        assert self.Q_noise.shape[0] == self.A.shape[0],                                \
            "Q must have the same dimensions as A"

        assert self.R_noise.shape[0] == self.R_noise.shape[1],                          \
            "R must be square"
        assert self.R_noise.shape[0] == self.C.shape[0],                                \
            "R must have the same dimensions as C"

        assert self.K.shape[0] == self.B.shape[1],                                      \
            "K must have the same number of rows as there are inputs"
        assert self.K.shape[1] == self.A.shape[0],                                      \
            "K must have the same number of columns as there are states"

        assert self.L.shape[0] == self.A.shape[0],                                      \
            "L must have the same number of rows as there are states"
        assert self.L.shape[1] == self.C.shape[0],                                      \
            "L must have the same number of columns as there are sensor inputs"


default_gains = StateSpaceGains(*([np.zeros((1, 1))]*8), 1., 'default')


class GainsList(object):

    def __init__(self, gains=[default_gains]):

        assert isinstance(gains, list) and isinstance(gains[0], StateSpaceGains)        \
            or isinstance(gains, StateSpaceGains),                                      \
            "Gains must be an instance of StateSpaceGains or a list of StateSpaceGains"

        self.gains_list = gains

    def add_gains(self, gains):

        assert isinstance(gains, list) and isinstance(gains[0], StateSpaceGains)        \
            or isinstance(gains, StateSpaceGains),                                      \
            "Gains must be an instance of StateSpaceGains or a list of StateSpaceGains"

        if isinstance(gains, list):
            self.gains_list += gains
        else:
            self.gains_list.append(gains)

    def get_gains(self, index: int):
        return self.gains_list[index]
