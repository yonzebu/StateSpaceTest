
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.0010553111479478304},
         {0.0, 7.673511273669439e-05}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.009048387850346559},
         {1.0115157353079955}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{651.8986469044033, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0.00010000009375379122, 5.568408094920313e-09},
         {5.568408094914818e-09, 5.276960636838686e-06}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.1}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{36.57876858310576, -0.12941187719181202}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.001530388163724601},
         {6.524412163983199e-12}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.008842827862079163, 0.9885362646972027}}
    );
    public static final Matrix N = new Matrix( new double[][]
        {{-0.05611112825099912}}
    );
    public static final Matrix U_min = new Matrix( new double[][]
        {{-10.0}}
    );
    public static final Matrix U_max = new Matrix( new double[][]
        {{10.0}}
    );
    public static final double dt = 0.01;
    
    public static StateSpaceGains kMotorGains = new StateSpaceGains(A, B, C, D, Q_noise, R_noise,
                                                                K, L, Kff, N, dt); 

}
            