
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.009999967506410714},
         {0.0, 0.9999935012891817}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{2.6624942323847784e-06},
         {0.0005324982697163706}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{65.18986469044033, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0.0, 0.0},
         {0.0, 0.0}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.1}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{764.793302491529, 783.3354284407666}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0},
         {0.0}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{9.389477304765554, 1877.8934269654183}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-10.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{10.0}}
    );
    public static final double dt = 0.01;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, dt); 

}
            