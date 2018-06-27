import numpy as np
from utilities.gains_writer import GainsWriter, numpy_to_jama_matrix
from utilities.state_space_gains import GainsList, StateSpaceGains, default_gains
from utilities.state_space_utils import observability, controllability
from robot import motor


OUT_DIR = '.\\src\\main\\java\\frc\\team687\\robot\\constants\\'

A = np.asmatrix([
    [1, 3, 0],
    [3, 2, 1],
    [2, 0, 0]
])

B = np.asmatrix([
    [1, 0],
    [2, 3],
    [0, 1]
])

C = np.asmatrix([
    [0, 1, 0],
    [2, 0, 3]
])

D = np.asmatrix([
    [1, 2],
    [2, 0]
])

Q_noise = np.zeros((3, 3))
R_noise = np.zeros((2, 2))
K = np.zeros((2, 3))
L = np.zeros((3, 2))


writer = GainsWriter(motor.create_gains())

writer.write(OUT_DIR, 0)
