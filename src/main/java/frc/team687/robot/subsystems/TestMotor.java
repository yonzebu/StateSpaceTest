package frc.team687.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Jama.Matrix;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team687.robot.constants.FlywheelGains;
import frc.team687.robot.constants.MotorGains;
import frc.team687.robot.constants.TestMotorConstants;
import frc.team687.utilities.statespace.ControllerSubsystem;
import frc.team687.utilities.statespace.NonlinearCompensator;
import frc.team687.utilities.statespace.StateSpaceGains;
import frc.team687.utilities.statespace.JamaUtils;

public class TestMotor extends ControllerSubsystem{

    private TalonSRX m_motor;

    private Matrix m_currentGoal;
    private NonlinearCompensator m_nonlinearCompensator;

    private String m_filePath1 = "/media/sda1/logs/";
    private String m_filePath2 = "/home/lvuser/logs/";
    private File m_file;
    private FileWriter m_writer;
    private boolean writeException = false;
    private double m_logStartTime = 0;

    public TestMotor() {
        super(MotorGains.kMotorGains, 
                MotorGains.U_min, MotorGains.U_max, TestMotorConstants.kInitialState,
                new Matrix(1,1), TestMotorConstants.kGainsIndex);

        this.m_motor = new TalonSRX(14);

        this.m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        this.m_motor.setSensorPhase(true);
        this.m_motor.setInverted(false);
        this.m_motor.setNeutralMode(NeutralMode.Coast);

        this.m_nonlinearCompensator = (x_hat) -> JamaUtils.matrixFromDouble(0);

        super.setInverted(false);
        
    }

    public void setEstimate(Matrix estimate) {
        this.setXHat(estimate);
    }

    private void setPercentOutput(double percentOutput) {
        this.m_motor.set(ControlMode.PercentOutput, percentOutput);
    }

    private void setVoltage(double voltage) {
        this.setPercentOutput(voltage / this.m_motor.getBusVoltage());
    }

    private void setVoltageWithCompensator(double voltage) {
        this.m_motor.set(ControlMode.PercentOutput, voltage / this.m_motor.getBusVoltage(), 
        DemandType.ArbitraryFeedForward, this.m_nonlinearCompensator.operation(this.getXHat()).get(0, 0));
    }

    public double getEstimatedAngle() {
        return this.getXHat().get(0, 0);
    }

    public double getEstimatedSpeed() {
        return this.getXHat().get(1, 0);
    }

    public double getEncoderSpeedTicks() {
        return this.m_motor.getSelectedSensorVelocity(0);
    }

    public double getEncoderPositionTicks() {
        return this.m_motor.getSelectedSensorPosition(0);
    }

    public double getCurrent() {
        return this.m_motor.getOutputCurrent();
    }

    public double getVoltage() {
        return this.m_motor.getMotorOutputVoltage();
    }

    public void resetEncoder() {
        this.m_motor.setSelectedSensorPosition(0, 0, 0);
    }

    public void configNonlinearCompensator(NonlinearCompensator nonlinearCompensator) {
        m_nonlinearCompensator = nonlinearCompensator;
    }

    public void setGoal(Matrix goal) {
        this.m_currentGoal = goal;
    }

    public void trackGoal() {
        Matrix measurement = new Matrix(new double[][]{
            {this.getEncoderPositionTicks()},
            {this.getEncoderSpeedTicks()}
        });
        double voltageToSet = this.trackReference(this.m_currentGoal, measurement).get(0,0);
        this.setVoltage(voltageToSet);
    }

    public void setInput(Matrix input) {
        Matrix measurement = new Matrix(new double[][]{{this.getEncoderPositionTicks()}});
        this.updateWithInput(input, measurement);
        double voltageToSet = input.get(0, 0);
        this.setVoltage(voltageToSet);
    }

    @Override
    protected void initDefaultCommand() {
        // setDefaultCommand(new TrackReference(TestMotorConstants.kEquilibriumGoal));
        // setDefaultCommand(new TrackReference(TestMotorConstants.kDefaultGoal));
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Position", getEncoderPositionTicks());
        SmartDashboard.putNumber("Velocity", getEncoderSpeedTicks());
        SmartDashboard.putNumber("Current", getCurrent());
        SmartDashboard.putNumber("Voltage", getVoltage());
        SmartDashboard.putNumber("Angle?", getEncoderPositionTicks() * 6.28 / 4096);
        SmartDashboard.putNumber("Angular Velocity", getEncoderSpeedTicks() * 6.28 * 10.0 / 4096);
        SmartDashboard.putNumber("EstPosition", getEstimatedAngle());
        SmartDashboard.putNumber("EstVelocity", getEstimatedSpeed());
    }

    public void startLog() {
        // Check to see if flash drive is mounted.
        File logFolder1 = new File(m_filePath1);
        File logFolder2 = new File(m_filePath2);
        Path filePrefix = Paths.get("");
        if (logFolder1.exists() && logFolder1.isDirectory()) {
            filePrefix = Paths.get(logFolder1.toString(),
                "2018_11_16_StateSpaceTesting_InertiaWheel");
        } else if (logFolder2.exists() && logFolder2.isDirectory()) {
            filePrefix = Paths.get(logFolder2.toString(),
                "2018_11_16_StateSpaceTesting_InertiaWheel");
        } else {
            writeException = true;
        }
    
        if (!writeException) {
            int counter = 0;
            while (counter <= 99) {
                m_file = new File(filePrefix.toString() + String.format("%02d", counter) + ".csv");
                if (m_file.exists()) {
                    counter++;
                } else {
                    break;
                }
                if (counter == 99) {
                    System.out.println("file creation counter at 99!");
                }
            }
            try {
                m_writer = new FileWriter(m_file);
                m_writer.append("Time,Position,Velocity,Voltage,Current,Angle," + 
                "EstimatedPosition,EstimatedSpeed\n");
                m_logStartTime = Timer.getFPGATimestamp();
            } catch (IOException e) {
                e.printStackTrace();
                writeException = true;
            }
        }
        }
    
        public void stopLog() {
        try {
            if (null != m_writer)
            m_writer.close();
        } catch (IOException e) {
            e.printStackTrace();
            writeException = true;
        }
        }
    
        public void logToCSV() {
        if (!writeException) {
            try {
                double timestamp = Timer.getFPGATimestamp() - m_logStartTime;
                m_writer.append(String.valueOf(timestamp) + ","
                    + String.valueOf(getEncoderPositionTicks()) + "," + String.valueOf(getEncoderSpeedTicks()) + ","
                    + String.valueOf(getVoltage()) + "," + String.valueOf(getCurrent()) + ","
                    + String.valueOf(getEncoderPositionTicks() * 6.28 / 4096.0) + ","
                    + String.valueOf(getEstimatedAngle()) + "," + String.valueOf(getEstimatedSpeed()) + "\n");
                m_writer.flush();
            } catch (IOException e) {
                e.printStackTrace();
                writeException = true;
            }
        }
        }

}
