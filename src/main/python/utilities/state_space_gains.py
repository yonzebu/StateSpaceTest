import numpy as np
import scipy
import scipy.signal
from utilities.state_space_utils import check_validity, observability, controllability


class StateSpaceGains(object):

    def __init__(self, A, B, C, D, Q_noise, R_noise, K, L, Kff, dt: float, name: str):
        self.A = np.asmatrix(A)
        self.B = np.asmatrix(B)
        self.C = np.asmatrix(C)
        self.D = np.asmatrix(D)

        self.Q_noise = np.asmatrix(Q_noise)
        self.R_noise = np.asmatrix(R_noise)

        self.K = np.asmatrix(K)
        self.L = np.asmatrix(L)
        self.Kff = np.asmatrix(Kff)

        self.dt = dt

        self.name = name

        self.is_controllable = self.check_controllability()
        self.is_observable = self.check_observability()

        self.check_system_validity()

    def check_controllability(self) -> bool:
        return np.linalg.matrix_rank(controllability(self.A, self.B)) == self.A.shape[0]

    def check_observability(self) -> bool:
        return np.linalg.matrix_rank(observability(self.A, self.C)) == self.A.shape[0]

    def check_system_validity(self):
        check_validity(self.A, self.B, self.C, self.D, self.Q_noise, self.R_noise, self.K, self.L, self.Kff)

    def print_gains(self):
        print('A = ', '\n', self.A)
        print('B = ', '\n', self.B)
        print('C = ', '\n', self.C)
        print('D = ', '\n', self.D)

        print('Q_noise = ', '\n', self.Q_noise)
        print('R_noise = ', '\n', self.R_noise)
        print('K = ', '\n', self.K)
        print('L = ', '\n', self.L)
        print('Kff = ', '\n', self.Kff)


# All matrices are defaulted to 1x1 zero matrices, dt is defaulted to 1, and name is defaulted to 'default'
default_gains = StateSpaceGains(*([np.zeros((1, 1))]*9), 1., 'default')


class GainsList(object):
    """ A wrapper around a list of gains. This should really extend list or something, but I'm a bit lazy"""

    def __init__(self, gains=default_gains):

        assert isinstance(gains, list) and isinstance(gains[0], StateSpaceGains)        \
            or isinstance(gains, StateSpaceGains),                                      \
            "Gains must be an instance of StateSpaceGains or a list of StateSpaceGains"

        if isinstance(gains, StateSpaceGains):
            self.gains_list = [gains]
        else:
            self.gains_list = gains

    # I should really just make this be a subclass of list or something
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

    def __len__(self):
        return len(self.gains_list)
