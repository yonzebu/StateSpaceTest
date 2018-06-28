
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0000036009582518}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.00059166773194951}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{65.18986469044033}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.1}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{18591.521908274248}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{1.1047581661348985e-07}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{1690.1378020144168}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-10.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{10.0}}
    );
    public static final double dt = 0.01;
    
    public static StateSpaceGains MotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, dt); 

}
            