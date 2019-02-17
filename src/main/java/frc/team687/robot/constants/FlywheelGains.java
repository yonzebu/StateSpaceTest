
package frc.team687.robot.constants;

import Jama.Matrix;
import frc.team687.utilities.statespace.StateSpaceGains;

public class FlywheelGains {

    public static final Matrix A = new Matrix( new double[][]
        {{0.8892527713479534}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{1.4153150992568042}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{586.708782213963}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0.01782591982401014}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.5}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{0.42620491126079224}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.0015155394787197973}}
    );
    public static final Matrix Kff = new Matrix( new double[][]
        {{0.01613133703728962}}
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
            