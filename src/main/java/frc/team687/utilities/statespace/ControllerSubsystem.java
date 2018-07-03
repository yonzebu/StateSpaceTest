package frc.team687.utilities.statespace;

import Jama.Matrix;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team687.robot.constants.MotorGains;
import frc.team687.robot.constants.TestMotorConstants;

abstract public class ControllerSubsystem extends Subsystem {

    protected Matrix m_currentOutput;

    private StateSpaceGains[] m_gains;
    private StateSpaceController m_controller;
    private StateSpaceObserver m_observer;
    private int m_gainsIndex;

    public ControllerSubsystem(StateSpaceGains gains, Matrix U_min, Matrix U_max, Matrix initialState,
                               Matrix initialInput, int initialIndex) {
        this(new StateSpaceGains[]{gains}, U_min, U_max, initialState, initialInput, initialIndex);
    }

    public ControllerSubsystem(StateSpaceGains[] gains, Matrix U_min, Matrix U_max, Matrix initialState,
                                Matrix initialInput, int initialIndex) {
        this.m_gains = gains;
        this.m_controller = new StateSpaceController(this.m_gains, U_min, U_max);
        this.m_observer = new StateSpaceObserver(this.m_gains, initialState);
        this.m_gainsIndex = initialIndex;
        this.setGainsIndex(m_gainsIndex);

        this.m_currentOutput = initialInput;

    }

    protected void setGainsIndex(int newIndex) {
        this.m_controller.setGainsIndex(0);
        this.m_observer.setGainsIndex(0);
    }

    protected Matrix trackReference(Matrix reference, Matrix measurement) {
        Matrix estimatedState = this.m_observer.newStateEstimate(this.m_currentOutput, measurement);
        this.m_currentOutput = this.m_controller.getBoundedOutput(reference, estimatedState);
        return this.m_currentOutput;
    }

}
