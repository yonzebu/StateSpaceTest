package frc.team687.robot.constants;

import Jama.Matrix;

public class TestMotorConstants {

    public static final int kGainsIndex = 0;
    public static final Matrix kDefaultGoal = new Matrix( new double[][] {
            {10},
            {0}
    });
    public static final Matrix kEquilibriumGoal = new Matrix( new double[][] {
            {0},
            {0}
    });
    public static final Matrix kInitialState = new Matrix( new double[][] {
            {0},
            {0}
    });

}
