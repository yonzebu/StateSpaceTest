package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceObserver {

    private StateSpaceGains[] m_gains;
    private int m_selectedGainsIndex;

    private Matrix m_xHat, m_currentInput;

    public StateSpaceObserver(StateSpaceGains[] gains, Matrix initialState) {
        this.m_gains = gains;
        this.m_selectedGainsIndex = 0;

        this.m_xHat = initialState;
    }

    public void setGainsIndex(int index) {
        if (index > this.m_gains.length - 1) {
            this.m_selectedGainsIndex = index;
        } else {
            this.m_selectedGainsIndex = this.m_gains.length - 1;
        }
    }

    public Matrix newStateEstimate(Matrix u, Matrix y) {
        StateSpaceGains gains = this.m_gains[m_selectedGainsIndex];

        /*
        x_hat[k+1] = Ax_hat[k] + Bu[k] + L(y[k] - y_hat[k])
        x_hat[k+1] = Ax_hat[k] + Bu[k] + L(y[k] - C(x_hat[k]))
        x_hat[k+1] = Ax_hat[k] + Bu[k] + Ly[k] - LCx_hat[k]
        x_hat[k+1] = (A - LC)x_hat[k] + Bu[k] + Ly[k]
        */

        // (A - LC)x_hat[k]
        Matrix xHatTerm = gains.A.minus(gains.L.times(gains.C)).times(this.m_xHat);
        // Bu[k]
        Matrix Bu = gains.B.times(u);
        // Ly[k]
        Matrix Ly = gains.L.times(y);

        this.m_xHat = xHatTerm.plus(Bu.plus(Ly));
        return this.m_xHat;
    }

    public Matrix getCurrentStateEstimate() {
        return this.m_xHat;
    }

    public void setXHat(Matrix xHat) {
        this.m_xHat = xHat;
    }

}
