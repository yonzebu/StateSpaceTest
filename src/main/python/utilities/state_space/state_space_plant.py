import numpy as np
from utilities.state_space.state_space_gains import GainsList


class StateSpacePlant(object):
    
    def __init__(self, gains, x_initial):
        self.gains = gains
        self.gains_index = 0
        self.current_gains = self.gains.get_gains(self.gains_index)

        self.x = np.asmatrix(x_initial)
        self.y = self.current_gains.C * self.x
    
    def set_index(self, index):
        self.gains_index = index
        self.current_gains = self.gains.get_gains(index)
    
    def update(self, u):
        gains = self.current_gains

        process_noise = gains.Q_noise * np.random.randn(gains.A.shape[0], 1)
        sensor_noise = gains.R_noise * np.random.randn(gains.C.shape[0], 1)
        
        self.x = gains.A * self.x + gains.B * u + process_noise
        self.y = gains.C * self.x + gains.D * u + sensor_noise

        return self.y
