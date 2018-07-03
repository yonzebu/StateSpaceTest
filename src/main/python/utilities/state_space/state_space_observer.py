from utilities.state_space.state_space_gains import GainsList


class StateSpaceObserver(object):

    def __init__(self, gains: GainsList, x_hat_initial):
        self.gains = gains
        self.gains_index = 0
        self.current_gains = self.gains.get_gains(self.gains_index)

        self.x_hat = x_hat_initial

    def set_index(self, index: int):
        self.gains_index = index
        self.current_gains = self.gains.get_gains(index)

    def update(self, u, y):
        gains = self.current_gains

        self.x_hat = (gains.A - (gains.L * gains.C)) * self.x_hat + gains.B * u + gains.L * y

        return self.x_hat
