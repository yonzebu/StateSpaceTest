package frc.team687.utilities.statespace;

import Jama.Matrix;

public class StateSpaceController {

    private StateSpaceGains[] m_gains;
    private Matrix u_min, u_max;

    private int selectedGainsIndex;

    public StateSpaceController(StateSpaceGains[] gains, Matrix uMin, Matrix uMax) {
        this.m_gains = gains;
        this.u_min = uMin;
        this.u_max = uMax;
        this.selectedGainsIndex = 0;
    }

    public void setGainsIndex(int index) {
        assert index < this.m_gains.length : "Gains index must be in bounds";
        this.selectedGainsIndex = index;
    }

}
