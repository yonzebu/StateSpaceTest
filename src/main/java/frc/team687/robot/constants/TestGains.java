package frc.team687.robot.constants;

import Jama.Matrix;

public class TestGains {

    public static final Matrix A = new Matrix( new double[][]
        {{1, 3, 0},
         {3, 2, 1},
         {2, 0, 0}}
    );
    public static final Matrix B = new Matrix( new double[][]
        {{1, 0},
         {2, 3},
         {0, 1}}
    );
    public static final Matrix C = new Matrix( new double[][]
        {{0, 1, 0},
         {2, 0, 3}}
    );
    public static final Matrix D = new Matrix( new double[][]
        {{1, 2},
         {2, 0}}
    );
    public static final Matrix Q_noise = new Matrix( new double[][]
        {{0., 0., 0.},
         {0., 0., 0.},
         {0., 0., 0.}}
    );
    public static final Matrix R_noise = new Matrix( new double[][]
        {{0., 0.},
         {0., 0.}}
    );
    public static final Matrix K = new Matrix( new double[][]
        {{0., 0., 0.},
         {0., 0., 0.}}
    );
    public static final Matrix L = new Matrix( new double[][]
        {{0., 0.},
         {0., 0.},
         {0., 0.}}
    );

}
            