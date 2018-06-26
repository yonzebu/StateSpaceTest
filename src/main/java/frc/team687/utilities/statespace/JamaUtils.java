package frc.team687.utilities.statespace;

import Jama.EigenvalueDecomposition;
import Jama.Matrix;
import frc.team687.utilities.MathUtils;


public class JamaUtils {

    // Returns e^(N * scalar)
    public static Matrix expm(Matrix N, double scalar) {
        if (isSquare(N)) {
            EigenvalueDecomposition E = N.eig();

            Matrix V = E.getV();

            if (MathUtils.deadband(V.det(), Math.pow(10, -13)) == 0) {
                int scale = 0;
                double scaledNorm = scalar * N.normInf();

                while (!( scaledNorm < 1)) {
                    scaledNorm *= .5;
                    scale++;
                }
                Matrix scaledN = N.times(Math.pow(2, -scale) * scalar);

                // Woo Taylor series because Pade approximation is hard and probably would actually work but nah
                Matrix expN;
                expN = expmTaylorSeriesApprox(scaledN);

                for (int i = 0; i < scale; i++) {
                    expN = expN.times(expN);
                }
                return expN;

            } else {
                Matrix D = E.getD();
                D.timesEquals(scalar);
                for (int i = 0; i < D.getRowDimension(); i++) {
                    D.set(i, i, Math.exp(D.get(i, i)));
                }


                return (V.times(D.times(V.inverse())));
            }

        } else {
            throw new RuntimeException("Matrix must be square for matrix exponential");
        }
    }

    public static double[] diagonals(Matrix N) {
        int n = N.getRowDimension();
        int m = N.getColumnDimension();
        int leastDimension = (n > m) ? m : n;

        double[] result = new double[leastDimension];
        for (int i = 0; i < leastDimension; i++) {
            result[i] = N.get(i, i);
        }

        return result;
    }

    public static Matrix diag(double[] entries) {
        Matrix result = new Matrix(entries.length, entries.length);
        for (int i = 0; i < entries.length; i++) {
            result.set(i, i, entries[i]);
        }

        return result;
    }

    /* Raises a matrix N to power. Negative powers are treated as raising the matrix's inverse to the absolute value of
       power. Raising a matrix to the 0th power returns the identity matrix. Only works on square matrices.
    */
    public static Matrix powm(Matrix N, int power) {
        if (isSquare(N)) {
            Matrix result = Matrix.identity(N.getRowDimension(), N.getColumnDimension());
            if (power >= 0) {
                for (int i = 0; i < power; i++) {
                    result = result.times(N);
                }
            } else {
                result = powm(N.inverse(), Math.abs(power));
            }
            return result;
        } else {
            throw new RuntimeException("Matrix must be square to be raised to a power");
        }
    }

    public static boolean isApproximatelyZero(Matrix N, double deadband) {
        for (int i = 0; i < N.getRowDimension(); i++) {
            for (int j = 0; j < N.getColumnDimension(); j++) {
                if (MathUtils.deadband(N.get(i, j), deadband) != 0) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean isSquare(Matrix N) {
        return N.getColumnDimension() == N.getRowDimension();
    }

    public static void printMatrix(Matrix N) {
        N.print(2, 5);
    }

    private static Matrix expmTaylorSeriesApprox(Matrix N) {
        Matrix currentMatrix = Matrix.identity(N.getRowDimension(), N.getColumnDimension());
        Matrix sum = currentMatrix;
        int currentTerm = 0;

        while (!isApproximatelyZero(currentMatrix, Math.pow(10, -15))) {
            currentTerm++;
            currentMatrix = powm(N, currentTerm).times(1 / MathUtils.factorial(currentTerm));

            sum.plusEquals(currentMatrix);
        }

        return sum;
    }

}