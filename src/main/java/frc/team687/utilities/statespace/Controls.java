package frc.team687.utilities.statespace;

import Jama.Matrix;
import frc.team687.utilities.statespace.JamaUtils;


public class Controls {


    public static StateSpaceGains continuousToDiscrete(Matrix A, Matrix B, Matrix C, Matrix D, Matrix Q, Matrix R, double dt) {
        int n = A.getColumnDimension();
        int m = B.getColumnDimension();

        Matrix F = new Matrix(n+m, n+m);
        F.setMatrix(0, n-1, 0, n-1, A);
        F.setMatrix(0, n-1, n, n+m-1, B);
        Matrix G = JamaUtils.expm(F, dt);

        Matrix Ad = G.getMatrix(0, n-1, 0, n-1);
        Matrix Bd = G.getMatrix(0, n-1, n, n+m-1);
        Matrix Cd = C;
        Matrix Dd = D;

        // A and Q must share dimensions, so n+n instead of using Q's dimensions
        Matrix H = new Matrix(n+n, n+n);
        H.setMatrix(0, n-1, 0, n-1, A.times(-1));
        H.setMatrix(0, n-1, n, n+n-1, Q);
        H.setMatrix(n, n+n-1, n, n+n-1, A.transpose());
        Matrix J = JamaUtils.expm(H, dt);
        Matrix Qd = (J.getMatrix(0, n-1, n, n+n-1)).times(Ad);
        Matrix Rd = R.times(1 / dt);

        // This is LQR yay that won't be traumatizing at all
        Matrix K = new Matrix(B.getColumnDimension(), C.getRowDimension());

        // I think calculating Kalman filter gain is roughly LQR but with A and C or something?
        // If so, this too won't be traumatizing at all, I'm certain
        Matrix L = new Matrix(A.getRowDimension(), C.getRowDimension());

        return new StateSpaceGains(Ad, Bd, Cd, Dd, Qd, Rd, K, L, dt);
    }

}
