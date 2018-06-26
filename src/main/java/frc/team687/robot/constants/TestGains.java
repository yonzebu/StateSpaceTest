package frc.team687.robot.constants;

import Jama.Matrix;

public class TestGains {

    public static Matrix A = new Matrix( new double[][]
        {{1, 3, 0},
         {3, 2, 1},
         {2, 0, 0}}
    );
    public static Matrix B = new Matrix( new double[][]
        {{1, 0},
         {2, 3},
         {0, 1}}
    );
    public static Matrix C = new Matrix( new double[][]
        {{0, 1, 0},
         {2, 0, 3}}
    );
    public static Matrix D = new Matrix( new double[][]
        {{1, 2},
         {2, 0}}
    );
    public static Matrix Q_noise = new Matrix( new double[][]
        {{0., 0., 0.},
         {0., 0., 0.},
         {0., 0., 0.}}
    );
    public static Matrix R_noise = new Matrix( new double[][]
        {{0., 0.},
         {0., 0.}}
    );
    public static Matrix K = new Matrix( new double[][]
        {{0., 0., 0.},
         {0., 0., 0.}}
    );
    public static Matrix L = new Matrix( new double[][]
        {{0., 0.},
         {0., 0.},
         {0., 0.}}
    );

}
            