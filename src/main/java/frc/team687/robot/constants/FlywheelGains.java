
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class FlywheelGains {

    public static final Matrix A = new Matrix( new double[][]
        {{0.9839263001485526}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.8803480323820759}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{65.18986469044033}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0.00019679387208189632}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.05}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{0.07970320501091194}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.014283628048590903}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.008735776946698805}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-1.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{1.0}}
    );
    public static final double dt = 0.02;
    
    public static StateSpaceGains kFlywheelGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, dt); 

}
            