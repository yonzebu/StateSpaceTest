package frc.team687.robot.constants;

import Jama.Matrix;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0000036}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.00059167}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{0.01533981}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{3.60095177e-08}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.1}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{0.106268}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{3.08673796}}
    );

}
            