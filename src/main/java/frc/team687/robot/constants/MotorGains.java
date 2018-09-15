
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.019870149814927702},
         {0.0, 0.9870431442789682}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.004978334802892345},
         {0.49675374537319267}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{651.8986469044033, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{2.002640741616894, 0.1974114268338357},
         {0.19741142683383572, 19.741422482774052}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.05}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{198.2873826668394, 3.623328828294932}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.00161004855233999},
         {0.003778645393068377}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.020172428447311527, 2.0128677120415355}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-0.3041690354910598}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-6.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{6.0}}
    );
    public static final double dt = 0.02;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, N, dt); 

}
            