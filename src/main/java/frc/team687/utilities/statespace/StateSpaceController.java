package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceController {

    private StateSpaceGains[] m_gains;
    private Matrix m_U_min, m_U_max;

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

    public Matrix getDesiredOutput(Matrix reference, Matrix estimatedState) {
        StateSpaceGains currentGains = this.m_gains[this.m_selectedGainsIndex];

        // Discrete-time reference tracking feedforward input: u_ff[k] = pinv(B) * (x[k+1] - Ax^[k]),
        // where x[k+1] = reference
        Matrix Uff = currentGains.Kff.times(reference.plus(currentGains.A.times(estimatedState)));

        // Discrete-time, reference tracking, not including feedforward input: u_c = -K * (x^ - r) = K * (r - x^)
        Matrix Uc = currentGains.K.times(reference.minus(estimatedState));

        // u = uc + uff
        return Uc.plus(Uff);
    }

    public Matrix getBoundedOutput(Matrix reference, Matrix estimatedState) {
        return JamaUtils.boundMatrix(getDesiredOutput(reference, estimatedState), this.m_U_min, this.m_U_max);
    }

}
