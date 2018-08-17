
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.0007958981365472213},
         {0.0, 0.6216589215919667}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.0002898175849505963},
         {0.5372312421693742}}
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
        {{1.0}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{688.7164612875005, 1.1578939034780324}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0},
         {0.0}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.001004158075752992, 1.8613952996092864}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-1.0564778199156108}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-10.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{10.0}}
    );
    public static final double dt = 0.001;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, N, dt); 

}
            