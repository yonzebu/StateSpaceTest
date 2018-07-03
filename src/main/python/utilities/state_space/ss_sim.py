from utilities.state_space.state_space_controller import StateSpaceController
from utilities.state_space.state_space_gains import GainsList, StateSpaceGains
from utilities.state_space.state_space_observer import StateSpaceObserver
from utilities.state_space.state_space_plant import StateSpacePlant
import numpy as np
import matplotlib.pyplot as plt


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

    def plot(self, duration, plot_setttings, reference_calculator=lambda time: np.zeros((1, 1))):

        x_list = []
        v_list = []
        u_list = []
        y_list = []
        x_hat_list = []
        v_hat_list = []

        for t in np.arange(start=0., stop=duration, step=self.current_gains.dt):
            x, u, y, x_hat = self.update(reference_calculator(t))
            x_list += [x[0, 0]]
            v_list += [x[1, 0]]
            u_list += [u[0, 0]]
            y_list += [y[0, 0]]
            x_hat_list += [x_hat[0, 0]]
            v_hat_list += [x_hat[1, 0]]

        generated_vals = x_list, v_list, u_list, y_list, x_hat_list, v_hat_list
        # x, v, u, y, x_hat, v_hat = generated_vals
        for i, flag in enumerate(plot_setttings):
            if flag:
                plt.plot(generated_vals[i])
        plt.show()
