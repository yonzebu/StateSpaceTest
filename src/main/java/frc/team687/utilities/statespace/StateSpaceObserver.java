package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceObserver {

    private StateSpaceGains[] m_gains;
    private int m_selectedGainsIndex;

    private Matrix m_currentState;

    public StateSpaceObserver(StateSpaceGains[] gains, Matrix initialState) {
        this.m_gains = gains;
        this.m_selectedGainsIndex = 0;

        this.m_currentState = initialState;
    }

    public void setGainsIndex(int index) {
        if (index > this.m_gains.length - 1) {
            this.m_selectedGainsIndex = index;
        } else {
            this.m_selectedGainsIndex = this.m_gains.length - 1;
        }
    }

    public Matrix newStateEstimate(Matrix u, Matrix y) {
        StateSpaceGains currentGains = this.m_gains[m_selectedGainsIndex];

        Matrix eyeMinusKkC = Matrix.identity(currentGains.K.getRowDimension(), currentGains.K.getRowDimension())
                .minus(currentGains.L.times(this.m_gains[m_selectedGainsIndex].C));
        Matrix term1 = currentGains.L.times(y);
        Matrix term2 = eyeMinusKkC.times(currentGains.A.times(this.m_currentState));
        Matrix term3 = eyeMinusKkC.times(currentGains.B.times(u));

        this.m_currentState = term1.plus(term2.plus(term3));
        return this.m_currentState;
    }

}
