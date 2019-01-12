
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.01985410941968353},
         {0.0, 0.9854465019528078}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.007948609224360154},
         {0.792923494948611}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{651.8986469044033, 0.0},
         {0.0, 65.18986469044033}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0},
         {0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{2.6375453426983808e-08, 1.970928304243831e-06},
         {1.9709283042438314e-06, 0.00019709636048349474}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.05, 0.0},
         {0.0, 0.5}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{6.2416740832741375, 0.8208889618204167}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0008525644694598732, 0.00028453586765431925},
         {0.009070043650920857, 0.009554416918987209}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.4861398789599107, 0.48495494114546406}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-1.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{1.0}}
    );
    public static final double dt = 0.02;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, dt); 

}
            