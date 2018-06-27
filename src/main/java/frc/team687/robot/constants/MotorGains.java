package frc.team687.robot.constants;

import Jama.Matrix;

public class MotorGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1.0, 0.010000018004780453},
         {0.0, 1.0000036009582518}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{2.9583368842750783e-06},
         {0.00059166773194951}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{0.015339807878856412, 0.0}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{0.0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{5.000000000005403e-09, 1.0000000000021614e-06},
         {1.8004758842143096e-12, 3.600951768432511e-10}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0.1}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{0.9982789378308087, 5.8176705097118795}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{1.1809937130477757},
         {0.3137834023587967}}
    );

}
            