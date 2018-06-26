package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceGains {

    public final Matrix A, B, C, D, Q, R, K, L;

    public final double dt;


    // This is used specifically for discrete state space constants
    public StateSpaceGains(Matrix A, Matrix B, Matrix C, Matrix D,
                           Matrix Q, Matrix R, Matrix K, Matrix L, double dt) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;

        this.Q = Q;
        this.R = R;

        this.K = K;
        this.L = L;

        this.dt = dt;

        this.checkSystemValidity();

    }

    public void checkSystemValidity() {
        assert this.A.getColumnDimension() == this.A.getRowDimension() :
                "A must be square";

        assert this.B.getRowDimension() == this.A.getRowDimension() :
                "A and B must have the same number of rows";

        assert this.C.getColumnDimension() == this.A.getRowDimension() :
                "C must have as many columns as there are states";

        assert this.D.getRowDimension() == this.C.getRowDimension() :
                "C and D must have the same number of rows";
        assert this.D.getColumnDimension() == this.B.getColumnDimension() :
                "B and D must have the same number of columns";

        assert this.Q.getRowDimension() == this.Q.getColumnDimension() :
                "Q must be square";
        assert this.Q.getRowDimension() == this.A.getRowDimension() :
                "Q must have the same dimensions as A";

        assert this.R.getRowDimension() == this.R.getColumnDimension() :
                "R must be square";
        assert this.R.getRowDimension() == this.C.getRowDimension() :
                "R must have the same dimensions as C";

        assert this.K.getRowDimension() == this.D.getColumnDimension() :
                "K must have the same number of rows as there are inputs";
        assert this.K.getColumnDimension() == this.A.getRowDimension() :
                "K must have the same number of columns as there are states";

        assert this.L.getRowDimension() == this.A.getRowDimension() :
                "L must have the same number of rows as there are states";
        assert this.L.getColumnDimension() == this.C.getRowDimension() :
                "L must have the same number of columns as there are sensor inputs";

        // I need to figure out how to Q and R
    }

}
