import numpy as np
from utilities.state_space.state_space_gains import GainsList


class StateSpaceController(object):

    def __init__(self, gains: GainsList, u_initial, r_initial, u_max, u_min):
        self.gains = gains
        self.gains_index = 0
        self.current_gains = self.gains.get_gains(self.gains_index)

        self.u_max = u_max
        self.u_min = u_min

        self.u = u_initial
        self.r = r_initial

    def set_index(self, index: int):
        self.gains_index = index
        self.current_gains = self.gains.get_gains(index)

    def update_ff(self, mot_prof, estimated_state):
        raise NotImplementedError

    def bounded_update_ff(self, r, x_hat):
        return np.clip(self.update_ff(r, x_hat), self.u_min, self.u_max)

    def update_ff(self, r, x_hat):
        gains = self.current_gains

        uff = gains.Kff * (r - (gains.A * self.r))
        self.u = gains.K * (r - x_hat)

        return self.u + uff

    def bounded_update(self, r, x_hat):
        return np.clip(self.update(r, x_hat), self.u_min, self.u_max)

    def update(self, r, x_hat):
        gains = self.current_gains

        self.r = r
        self.u = gains.K * (r - x_hat)

        return self.u


