import numpy as np
from utilities.state_space.state_space_gains import GainsList


class StateSpaceController(object):

    def __init__(self, gains: GainsList, u_initial, u_max, u_min):
        self.gains = gains
        self.gains_index = 0
        self.current_gains = self.gains.get_gains(self.gains_index)

        self.u_max = u_max
        self.u_min = u_min

        self.u = u_initial

    def set_index(self, index: int):
        self.gains_index = index
        self.current_gains = self.gains.get_gains(index)

    def update_ff(self, mot_prof, estimated_state):
        raise NotImplementedError

    def bounded_update(self, r, x_hat):
        return np.clip(self.update(r, x_hat), self.u_min, self.u_max)

    def update(self, r, x_hat):
        gains = self.current_gains

        self.u = -gains.K * x_hat + gains.N * r

        return self.u


