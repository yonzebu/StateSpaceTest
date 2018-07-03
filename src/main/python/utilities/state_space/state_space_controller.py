import numpy as np
from utilities.state_space.state_space_gains import StateSpaceGains, GainsList
from utilities.state_space.state_space_observer import StateSpaceObserver
from utilities.state_space.state_space_plant import StateSpacePlant


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


class StateSpaceControlSim(object):

    def __init__(self, gains: GainsList, x_hat_initial, u_initial, x_initial, u_max, u_min):
        assert isinstance(gains, GainsList) or isinstance(gains, StateSpaceGains), \
            "Gains must be a list of gains or a state space gains object"
        if isinstance(gains, StateSpaceGains):
            self.gains = GainsList(gains)
        else:
            self.gains = gains

        self.gains_index = 0
        self.current_gains = self.gains.get_gains(self.gains_index)

        self.controller = StateSpaceController(gains=self.gains, u_initial=u_initial, u_max=u_max, u_min=u_min)
        self.observer = StateSpaceObserver(gains=self.gains, x_hat_initial=x_hat_initial)
        self.plant = StateSpacePlant(gains=self.gains, x_initial=x_initial)

        self.u = u_initial
        self.y = self.current_gains.C * x_initial
        self.x_hat = x_hat_initial

    def set_gains_index(self, index: int):
        self.gains_index = 0
        self.current_gains = self.gains.get_gains(self.gains_index)

        self.controller.set_index(index)
        self.observer.set_index(index)
        self.plant.set_index(index)

    def update(self, r):
        self.y = self.plant.update(self.u)
        self.x_hat = self.observer.update(self.u, self.y)
        self.u = self.controller.bounded_update(r, self.x_hat)

        return self.plant.x, self.u, self.y, self.x_hat
