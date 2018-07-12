from utilities.state_space.state_space_controller import StateSpaceController
from utilities.state_space.state_space_gains import GainsList, StateSpaceGains
from utilities.state_space.state_space_observer import StateSpaceObserver
from utilities.state_space.state_space_plant import StateSpacePlant
import numpy as np
import matplotlib.pyplot as plt


class StateSpaceControlSim(object):

    def __init__(self, gains, x_hat_initial, u_initial, x_initial, u_max, u_min):
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

        self.num_states = self.current_gains.A.shape[0]
        self.num_inputs = self.current_gains.B.shape[1]
        self.num_sensor_inputs = self.current_gains.C.shape[0]

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

    def plot(self, duration, plot_setttings,
             reference_calculator=(lambda time: np.zeros((1, 1)))):

        # x, then u, then y, then x_hat
        x_list = [[]] * self.num_states
        u_list = [[]] * self.num_inputs
        y_list = [[]] * self.num_sensor_inputs
        x_hat_list = [[]] * self.num_states

        for t in np.arange(start=0., stop=duration, step=self.current_gains.dt):
            x, u, y, x_hat = self.update(reference_calculator(t))
            for state_num in range(self.num_states):
                x_list[state_num] = x_list[state_num] + [x[state_num, 0]]
            for input_num in range(self.num_inputs):
                u_list[input_num] = u_list[input_num] + [u[input_num, 0]]
            for output_num in range(self.num_sensor_inputs):
                y_list[output_num] = y_list[output_num] + [y[output_num, 0]]
            for est_state_num in range(self.num_states):
                x_hat_list[est_state_num] = x_hat_list[est_state_num] + [x_hat[est_state_num, 0]]

        generated_vals = [[]] * (2*self.num_states + self.num_sensor_inputs + self.num_inputs)
        current_idx = 0
        for i in range(self.num_states):
            generated_vals[i] = x_list[i]
        current_idx += self.num_states

        for i in range(self.num_inputs):
            generated_vals[current_idx + i] = u_list[i]
        current_idx += self.num_inputs

        for i in range(self.num_sensor_inputs):
            generated_vals[current_idx + i] = y_list[i]
        current_idx += self.num_sensor_inputs

        for i in range(self.num_states):
            generated_vals[current_idx + i] = x_hat_list[i]
        
        # x, u, y, x_hat, all expanded hopefully = generated_vals
        for i, flag in enumerate(plot_setttings):
            if flag:
                plt.plot(generated_vals[i])
        plt.show()
