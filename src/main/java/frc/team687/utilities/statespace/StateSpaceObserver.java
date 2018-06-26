package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceObserver {

    private StateSpaceGains[] gains;
    private int selectedGainsIndex;

    public Matrix currentState;

    public StateSpaceObserver(StateSpaceGains[] gains, Matrix initialState) {
        this.gains = gains;
        this.selectedGainsIndex = 0;

        this.currentState = initialState;
    }

    public void setGainsIndex(int index) {
        if (index > this.gains.length - 1) {
            this.selectedGainsIndex = index;
        } else {
            this.selectedGainsIndex = this.gains.length - 1;
        }
    }

    public Matrix newStateEstimate(Matrix u, Matrix y) {
        StateSpaceGains currentGains = this.gains[selectedGainsIndex];

        Matrix eyeMinusKkC = Matrix.identity(currentGains.K.getRowDimension(), currentGains.K.getRowDimension())
                .minus(currentGains.L.times(this.gains[selectedGainsIndex].C));
        Matrix term1 = currentGains.L.times(y);
        Matrix term2 = eyeMinusKkC.times(currentGains.A.times(this.currentState));
        Matrix term3 = eyeMinusKkC.times(currentGains.B.times(u));

        this.currentState = term1.plus(term2.plus(term3));
        return this.currentState;
    }

}
