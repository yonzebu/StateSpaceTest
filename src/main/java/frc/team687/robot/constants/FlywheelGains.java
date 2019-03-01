
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class FlywheelGains {

    public static final Matrix A = new Matrix( new double[][]
        {{0.9102}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.9901}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{1}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{1.374}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{0.6087503389054563}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.5911953789506351}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-12.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{12.0}}
    );
    public static final double dt = 0.02;
    
    public static StateSpaceGains kFlywheelGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, dt); 

}
            