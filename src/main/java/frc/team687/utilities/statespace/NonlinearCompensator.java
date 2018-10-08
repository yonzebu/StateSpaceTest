package frc.team687.utilities.statespace;

import Jama.Matrix;

public interface NonlinearCompensator {
    public Matrix operation(Matrix x_hat);
}