package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceController {

    private StateSpaceGains[] m_gains;
    private Matrix m_uMin, m_uMax;

    private int m_selectedGainsIndex;

    public StateSpaceController(StateSpaceGains[] gains, Matrix uMin, Matrix uMax) {
        this.m_gains = gains;
        this.m_uMin = uMin;
        this.m_uMax = uMax;
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

        // Discrete-time, reference tracking, not including feedforward input: u_c = K * (x^ - r)
        Matrix Uc = currentGains.K.times(estimatedState.minus(reference));

        // u = uc + uff
        return Uc.plus(Uff);
    }

    public Matrix getBoundedOutput(Matrix reference, Matrix estimatedState) {
        return JamaUtils.boundMatrix(getDesiredOutput(reference, estimatedState), this.m_uMin, this.m_uMax);
    }

}
