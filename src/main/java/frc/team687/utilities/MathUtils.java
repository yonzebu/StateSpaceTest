package frc.team687.utilities;

public class MathUtils {

    public static double deadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    public static double factorial(int n) {
        if (n >= 0) {
            double product = 1;
            for (int i = 1; i < n + 1; i++) {
                product *= i;
            }
            return product;
        } else {
            throw new IllegalArgumentException("Factorials must be non-negative until the Gamma function is implemented.");
        }
    }

}
