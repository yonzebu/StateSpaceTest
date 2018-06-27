package frc.team687.robot.constants;

import Jama.Matrix;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0000036009582518}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{0.00059166773194951}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{0.015339807878856412}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{3.600951768432511e-09}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.1}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{37.774015311026254}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0.9925925094407523}}
    );

}
            