package frc.team687.robot.subsystems;

import Jama.Matrix;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team687.robot.commands.TrackReference;
import frc.team687.robot.constants.TestMotorConstants;
import frc.team687.utilities.statespace.ControllerSubsystem;
import frc.team687.utilities.statespace.StateSpaceController;
import frc.team687.utilities.statespace.StateSpaceGains;
import frc.team687.utilities.statespace.StateSpaceObserver;
import frc.team687.robot.constants.MotorGains;

public class TestMotor extends ControllerSubsystem {

    private TalonSRX m_motor;

    private Matrix m_currentGoal;

    public TestMotor() {
        super(MotorGains.kMotorGains, MotorGains.U_min, MotorGains.U_max, TestMotorConstants.kInitialState,
                new Matrix(1,1), TestMotorConstants.kGainsIndex);

        this.m_motor = new TalonSRX(0);

        this.m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        this.m_motor.setSensorPhase(false);
        this.m_motor.setInverted(false);
        this.m_motor.setNeutralMode(NeutralMode.Coast);
        
    }

    public void setPercentOutput(double percentOutput) {
        this.m_motor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setVoltage(double voltage) {
        this.m_motor.set(ControlMode.PercentOutput, voltage / this.m_motor.getBusVoltage());
    }

    public double getEncoderSpeedTicks() {
        return this.m_motor.getSelectedSensorVelocity(0);
    }

    public double getEncoderPositionTicks() {
        return this.m_motor.getSelectedSensorPosition(0);
    }

    public void resetEncoder() {
        this.m_motor.setSelectedSensorPosition(0, 0, 0);
    }

    public void setGoal(Matrix goal) {
        this.m_currentGoal = goal;
    }

    public void trackGoal() {
        Matrix measurement = new Matrix(new double[][]{{this.getEncoderPositionTicks()}});
        this.setVoltage(this.trackReference(this.m_currentGoal, measurement).get(0,0));
    }

    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new TrackReference(TestMotorConstants.kEquilibriumGoal));
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Position", getEncoderPositionTicks());
        SmartDashboard.putNumber("Veclocity", getEncoderSpeedTicks());
    }

}
