package frc.team687.robot.commands;

import Jama.Matrix;

public interface InputCalculator {
    public Matrix operation(double t);
}