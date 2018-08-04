
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.0010699959597150442},
         {0.0, 7.626329098890846e-09}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.01367214389660196},
         {0.7222472728076548}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{651.8986469044033, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0.0, 0.0},
         {0.0, 0.0}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.05}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{25.61449616567387, -0.2079690369834371}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0},
         {0.0}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.026200477113035397, 1.3840713851653}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-0.03929214500951415}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-10.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{10.0}}
    );
    public static final double dt = 0.02;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, N, dt); 

}
            