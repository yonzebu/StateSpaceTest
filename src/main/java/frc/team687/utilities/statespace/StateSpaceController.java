package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceController {

    private StateSpaceGains[] m_gains;
    private Matrix m_U_min, m_U_max;
    private StateSpaceObserver m_observer;

    private int m_selectedGainsIndex;

    public StateSpaceController(StateSpaceGains[] gains, Matrix U_min, Matrix U_max) {
        this.m_gains = gains;
        this.m_U_min = U_min;
        this.m_U_max = U_max;
        this.m_selectedGainsIndex = 0;
    }

    public void setGainsIndex(int index) {
        assert index < this.m_gains.length : "Gains index must be in bounds";
        this.m_selectedGainsIndex = index;
    }

    public Matrix getFeedforwardOutput(Matrix nextGoal, Matrix estimatedState) {
        StateSpaceGains currentGains = this.m_gains[this.m_selectedGainsIndex];

        // Discrete-time reference tracking feedforward input: u_ff[k] = pinv(B) * (x[k+1] - Ax^[k]),
        // where x[k+1] = reference
        Matrix Uff = currentGains.Kff.times(nextGoal.plus(currentGains.A.times(estimatedState)));

        // Discrete-time, reference tracking, not including feedforward input: u_c = -K * (x^ - r) = K * (r - x^)
        Matrix Uc = currentGains.K.times(nextGoal.minus(estimatedState));

        // u = uc + uff
        return Uc.plus(Uff);
    }

    public Matrix getReferenceTrackingOutput(Matrix reference, Matrix estimatedState) {
        StateSpaceGains currentGains = this.m_gains[this.m_selectedGainsIndex];

        Matrix referenceInput = currentGains.N.times(reference);


        Matrix controlLawInput = currentGains.K.times(estimatedState);

        // u = -K*x + N*r
        referenceInput.minusEquals(controlLawInput);
        JamaUtils.printMatrix(estimatedState);

        return controlLawInput;
    }

    public Matrix boundOutput(Matrix output) {
        return JamaUtils.boundMatrix(output, this.m_U_min, this.m_U_max);
    }

    public Matrix getBoundedOutput(Matrix reference, Matrix estimatedState) {
        return this.boundOutput(getReferenceTrackingOutput(reference, estimatedState));
    }

}
