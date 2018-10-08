
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.019837987121942537},
         {0.0, 0.983842577723303}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.008827006200120462},
         {0.8803106785362002}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{651.8986469044033, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{2.1074673068633687e-08, 1.5741829322014317e-06},
         {1.5741829322014317e-06, 0.00015742177403534467}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.05}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{5.991958837354846, 0.7761039687657408}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0016702324390587306},
         {0.02971492356522151}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.011389320861555712, 1.1358483894081515}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-0.009191549738303915}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-1.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{1.0}}
    );
    public static final double dt = 0.02;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, N, dt); 

}
            