package frc.team687.robot.subsystems;

import Jama.Matrix;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team687.robot.commands.TrackReference;
import frc.team687.robot.constants.MotorGains;
import frc.team687.utilities.statespace.StateSpaceController;
import frc.team687.utilities.statespace.StateSpaceGains;
import frc.team687.utilities.statespace.StateSpaceObserver;

public class TestMotor extends Subsystem {

    private StateSpaceController m_controller;
    private StateSpaceGains[] m_gains;
    private StateSpaceObserver m_observer;

    private TalonSRX m_motor;

    private Matrix m_currentGoal, m_currentState, m_currentInput;

    public static final int kGainsIndex = 0;
    public static final Matrix kDefaultGoal = new Matrix( new double[][] {{0}} );
    public static final Matrix kEquilibriumGoal = new Matrix( new double[][] {{0}} );
    public static final Matrix kInitialState = new Matrix( new double[][] {{0}} );

    public TestMotor() {
        this.m_gains = new StateSpaceGains[] {MotorGains.MotorGains};
        this.m_controller = new StateSpaceController(this.m_gains, MotorGains.U_min, MotorGains.U_max);
        this.m_controller.setGainsIndex(0);
        this.m_observer = new StateSpaceObserver(this.m_gains, kInitialState);
        this.m_observer.setGainsIndex(0);

        this.m_motor = new TalonSRX(0);

        this.m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        this.m_motor.setSensorPhase(false);
        this.m_motor.setInverted(false);
        this.m_motor.setNeutralMode(NeutralMode.Coast);

        this.m_currentState = kInitialState;
        this.m_currentInput = new Matrix( new double[][] {{0}} );
        
    }

    public void setPercentOutput(double percentOutput) {
        this.m_motor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setVoltage(double voltage) {
        this.m_motor.set(ControlMode.PercentOutput, voltage / 12.0);
    }

    public double getEncoderSpeedTicks() {
        return this.m_motor.getSelectedSensorVelocity(0);
    }

    public void setGoal(Matrix goal) {
        this.m_currentGoal = goal;
    }

    public void trackGoal() {
        this.m_currentState.set(0, 0, this.getEncoderSpeedTicks());
        this.m_currentState = this.m_observer.newStateEstimate(this.m_currentInput, this.m_currentState);
        double voltage = this.m_controller.getBoundedOutput(this.m_currentGoal, this.m_currentState).get(0, 0);
        this.setVoltage(voltage);
    }

    public StateSpaceGains gains() {
        return this.m_gains[kGainsIndex];
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new TrackReference(kEquilibriumGoal));
    }

}
