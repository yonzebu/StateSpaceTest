import numpy as np
from utilities.state_space_gains import GainsList


def numpy_to_jama_matrix(np_matrix):
    matrix = np.asmatrix(np_matrix)

    # Beginning brace for the double[][]
    output = '{'
    # Iterates through each entry in the matrix, adding commas after individual entries and braces at the beginning
    # and end of rows
    for i in range(matrix.shape[0]):
        output += '{'
        for j in range(matrix.shape[1]):
            output += str(matrix[i, j]) + ', '
        # This should remove the extra comma and space
        output = output[:-2]
        # The spacing should work, since this is always outputting to the same type of thing
        output += '},\n         '
    # This is to remove the extra padding and the newline character
    output = output[:-11]
    # Ending brace for the double[][]
    output += '}'
    return output


class GainsWriter(object):
    """ A class to handle writing gains to Java files"""

    def __init__(self, gains: GainsList=GainsList()):
        self.gains = gains

    def write_all(self, paths):
        assert isinstance(paths, list) and isinstance(paths[0], str) or \
            isinstance(paths, str)
        if isinstance(paths, str):
            paths = [paths]
        for path in paths:
            assert isinstance(path, str), 'Directory paths must be strings'
        assert len(paths) == len(self.gains), 'The number of paths must be equal to the number of gains lists'
        for i, path in enumerate(paths):
            self.write(path, i)

    def write(self, path: str, gains_index: int):
        current_gains = self.gains.get_gains(gains_index)

        current_name = current_gains.name

        current_A_data = numpy_to_jama_matrix(current_gains.A)
        current_B_data = numpy_to_jama_matrix(current_gains.B)
        current_C_data = numpy_to_jama_matrix(current_gains.C)
        current_D_data = numpy_to_jama_matrix(current_gains.D)
        current_Q_noise_data = numpy_to_jama_matrix(current_gains.Q_noise)
        current_R_noise_data = numpy_to_jama_matrix(current_gains.R_noise)
        current_K_data = numpy_to_jama_matrix(current_gains.K)
        current_L_data = numpy_to_jama_matrix(current_gains.L)
        current_Kff_data = numpy_to_jama_matrix(current_gains.Kff)
        current_u_min_data = numpy_to_jama_matrix(current_gains.u_min)
        current_u_max_data = numpy_to_jama_matrix(current_gains.u_max)
        current_dt_data = str(current_gains.dt)

        # Open the file given the path, and name, truncate it to clear it, and then write the data
        javafile = open(path + current_name + '.java', 'w')
        javafile.truncate()
        javafile.write('''
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class {name} {{

    public static final Matrix A = new Matrix( new double[][]
        {A_data}
    );
    public static final Matrix B = new Matrix( new double[][]
        {B_data}
    );
    public static final Matrix C = new Matrix( new double[][]
        {C_data}
    );
    public static final Matrix D = new Matrix( new double[][]
        {D_data}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {Q_noise_data}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {R_noise_data}
    );
    public static final Matrix K = new Matrix( new double[][]
        {K_data}
    );
    public static final Matrix L = new Matrix( new double[][]
        {L_data}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {Kff_data}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {u_min_data}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {u_max_data}
    );
    public static final double dt = {dt_data};
    
    public static StateSpaceGains k{name} = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, dt); 

}}
            '''.format(name=current_name, A_data=current_A_data, B_data=current_B_data,
                       C_data=current_C_data, D_data=current_D_data, Q_noise_data=current_Q_noise_data,
                       R_noise_data=current_R_noise_data, K_data=current_K_data, L_data=current_L_data,
                       Kff_data=current_Kff_data, u_min_data=current_u_min_data, u_max_data=current_u_max_data,
                       dt_data=current_dt_data,)
        )
        javafile.close()
