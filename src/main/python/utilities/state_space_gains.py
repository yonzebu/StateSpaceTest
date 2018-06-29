import numpy as np
from utilities.state_space_utils import check_validity, observability, controllability


class Gains(object):
    """ Base class for the two gains variants"""
    pass

class StateSpaceGains(Gains):

    def __init__(self, name: str, A, B, C, D, Q_noise, R_noise, K, L, Kff, u_min, u_max, dt: float):
        self.A = np.asmatrix(A)
        self.B = np.asmatrix(B)
        self.C = np.asmatrix(C)
        self.D = np.asmatrix(D)

        self.Q_noise = np.asmatrix(Q_noise)
        self.R_noise = np.asmatrix(R_noise)

        self.K = np.asmatrix(K)
        self.L = np.asmatrix(L)
        self.Kff = np.asmatrix(Kff)

        self.u_min = u_min
        self.u_max = u_max

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

        print('u_min = ', '\n', self.u_min)
        print('u_max = ', '\n', self.u_max)


class ContinuousGains(Gains):
    """ This is a dumb class which I probably auin't using"""

    def __init__(self, name: str, A, B, C, D, u_min, u_max, B_ref=None):

        self.A = np.asmatrix(A)
        self.B = np.asmatrix(B)
        self.C = np.asmatrix(C)
        self.D = np.asmatrix(D)

        # Hopefully B_ref * x + A*x = dx/dt = 0
        if B_ref is not None:
            self.B_ref = np.asmatrix(B_ref)
        else:
            self.B_ref = np.linalg.pinv(B) * -self.A

        self.u_min = np.asmatrix(u_min)
        self.u_max = np.asmatrix(u_max)

        self.name = name


# All matrices are defaulted to 1x1 zero matrices, dt is defaulted to 1, and name is defaulted to 'default'
default_gains = StateSpaceGains('default', *([np.zeros((1, 1))]*11), 1.)


class GainsList(object):
    """ A wrapper around a list of gains. This should really extend list or something, but I'm a bit lazy"""

    def __init__(self, gains=default_gains):

        assert isinstance(gains, list) and isinstance(gains[0], Gains)              \
            or isinstance(gains, Gains),                                            \
            "Gains must be an instance of Gains or a list of Gains"

        if isinstance(gains, Gains):
            self.gains_list = [gains]
        else:
            self.gains_list = gains

    # I should really just make this be a subclass of list or something
    def add_gains(self, gains):

        assert isinstance(gains, list) and isinstance(gains[0], Gains)              \
            or isinstance(gains, Gains),                                            \
            "Gains must be an instance of Gains or a list of Gains"

        if isinstance(gains, list):
            self.gains_list += gains
        else:
            self.gains_list.append(gains)

    def get_gains(self, index: int):
        return self.gains_list[index]

    def __len__(self):
        return len(self.gains_list)
