package frc.team687.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team687.robot.Robot;
import frc.team687.robot.constants.TestMotorConstants;

public class ResetMotorEncoder extends Command{

    private int m_counter;

    public ResetMotorEncoder() {
        requires(Robot.motor);
        m_counter = 0;
    }

    protected void initialize() {
        Robot.motor.setVoltage(0);

        SmartDashboard.putString("Current command", "ResetMotorEncoder");
    }

    protected void execute() {
        Robot.motor.resetEncoder();
        if (Robot.motor.getEncoderPositionTicks() == 1000) {
            this.m_counter++;
        } else {
            this.m_counter = 0;
        }
    }

    @Override
    protected boolean isFinished() {
        return m_counter > 4;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.motor.setGoal(TestMotorConstants.kEquilibriumGoal);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        this.end();
    }
}
