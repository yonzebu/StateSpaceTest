
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.049193691175100696},
         {0.0, 0.9679219550492804}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.030913127175293275},
         {1.2298422793775177}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{651.8986469044033, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0.0005000406631207223, 1.21000962571559e-06},
         {1.2100096257155903e-06, 4.8404672456697354e-05}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.02}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{6.017031715437111, 0.7984086292017798}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0015392193785951584},
         {0.00010590326590034832}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.020425363635785215, 0.8125989851659301}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-0.009230011051579109}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-6.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{6.0}}
    );
    public static final double dt = 0.05;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, N, dt); 

}
            