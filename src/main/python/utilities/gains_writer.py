import numpy as np
from utilities.state_space_gains import GainsList


def numpy_to_jama_matrix(np_matrix):
    # Removing all square brackets and replacing them with curly brackets. Singular ending braces get a comma the end
    curly_braces = str(np_matrix).replace('[', '{').replace(']]', '}}').replace(']', '},')
    # Adding commas in between numbers, and then filtering the spaces replaced to remove some excess commas and add tabs
    return curly_braces.replace(' ', ', ').replace(', {', '         {')


class GainsWriter(object):

    def __init__(self, gains: GainsList=GainsList()):
        self.gains = gains

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

        print(current_A_data, '\n', current_B_data, '\n', current_C_data, '\n', current_D_data, '\n',
              current_Q_noise_data, '\n', current_R_noise_data, '\n', current_K_data, '\n', current_L_data, '\n', )

        javafile = open(path + current_name + '.java', 'w')
        javafile.truncate()
        javafile.write(
            '''package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.

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
    
    public static final 

}}
            '''.format(name=current_name, A_data=current_A_data, B_data=current_B_data,
                       C_data=current_C_data, D_data=current_D_data, Q_noise_data=current_Q_noise_data,
                       R_noise_data=current_R_noise_data, K_data=current_K_data, L_data=current_L_data)
        )
        javafile.close()
