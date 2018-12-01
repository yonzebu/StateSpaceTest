
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.008326411545369602},
         {0.0, 0.12110428773226439}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.05300136163153747},
         {3.9904327331183826}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{651.8986469044033, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{7.193181124721804e-09, 3.4664564611432126e-07},
         {3.4664564611432126e-07, 4.667387842468647e-05}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.05}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{2.21276022504218, 0.04372777903032654}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0004960453823274704},
         {0.00047335719610875027}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.30806742848487134, 0.231941654476694}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-0.0033943316734122105}}
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
            