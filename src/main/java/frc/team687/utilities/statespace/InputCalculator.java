package frc.team687.utilities.statespace;

import Jama.Matrix;

public interface InputCalculator {
    public Matrix operation(double t);
}