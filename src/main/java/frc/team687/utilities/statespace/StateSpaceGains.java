package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceGains {

    public final Matrix A, B, C, D, Q, R, K, L, Kff, N;

    public final double dt;
    public final int n, p, q;


    // This is used specifically for discrete state space constants
    public StateSpaceGains(Matrix A, Matrix B, Matrix C, Matrix D, Matrix Q,
                           Matrix R, Matrix K, Matrix L, Matrix Kff, Matrix N,
                           double dt) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;

        this.Q = Q;
        this.R = R;

        this.K = K;
        this.L = L;
        this.Kff = Kff;
        this.N = N;

        this.dt = dt;

        // Number of states
        this.n = this.A.getRowDimension();
        // Number of inputs
        this.p = this.B.getColumnDimension();
        // Number of sensor inputs (measurements)
        this.q = this.C.getRowDimension();

        this.checkSystemValidity();

    }

    private void checkSystemValidity() {
        assert this.A.getColumnDimension() == this.A.getRowDimension() :
                "A must be square";

        assert this.B.getRowDimension() == this.n :
                "B must have the same number of rows as there are states";

        assert this.C.getColumnDimension() == this.n :
                "C must have as many columns as there are states";

        assert this.D.getRowDimension() == this.q :
                "D must have the same number of rows as there are sensor inputs";
        assert this.D.getColumnDimension() == this.p :
                "D must have the same number of columns as there are inputs";

        assert this.Q.getRowDimension() == this.Q.getColumnDimension() :
                "Q must be square";
        assert this.Q.getRowDimension() == this.n :
                "Q must have the same dimensions as A";

        assert this.R.getRowDimension() == this.R.getColumnDimension() :
                "R must be square";
        assert this.R.getRowDimension() == this.q :
                "R must have the same dimensions as C";

        assert this.K.getRowDimension() == this.p :
                "K must have the same number of rows as there are inputs";
        assert this.K.getColumnDimension() == this.n :
                "K must have the same number of columns as there are states";

        assert this.L.getRowDimension() == this.n :
                "L must have the same number of rows as there are states";
        assert this.L.getColumnDimension() == this.q :
                "L must have the same number of columns as there are sensor inputs";

        assert this.Kff.getColumnDimension() == this.n :
                "Kff must have the same number of columns as there are states";
        assert this.Kff.getRowDimension() == this.p :
                "Kff must have the same number of rows as there are inputs";

        assert this.N.getRowDimension() == this.p :
                "N must have the same number of rows as there are states";
        assert this.N.getColumnDimension() == this.q :
                "N must have the same number of columns as there are sensor inputs";
    }

}
