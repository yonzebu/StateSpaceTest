
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class FlywheelGains {

    public static final Matrix A = new Matrix( new double[][]
        {{0.12110428773226439}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{3.9904327331183826}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{65.18986469044033}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{4.667387842468647e-05}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.05}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{0.028556842066895335}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0014845864571035278}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.2357916590789855}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-0.003816660315033393}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-1.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{1.0}}
    );
    public static final double dt = 0.02;
    
    public static StateSpaceGains kFlywheelGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, N, dt); 

}
            