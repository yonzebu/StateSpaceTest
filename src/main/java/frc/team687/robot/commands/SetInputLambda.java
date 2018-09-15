package frc.team687.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team687.robot.Robot;
import frc.team687.robot.constants.TestMotorConstants;
import frc.team687.utilities.statespace.JamaUtils;

public class SetInputLambda extends Command{

    private InputCalculator m_inputCalculator;
    private Timer m_timer;

    public SetInputLambda(InputCalculator inputCalculator) {
        requires(Robot.motor);
        this.m_inputCalculator = inputCalculator;
        this.m_timer = new Timer();
    }

    protected void initialize() {
        Robot.motor.setInput(JamaUtils.matrixFromDouble(0));
        this.m_timer.stop();
        this.m_timer.reset();
        this.m_timer.start();

        SmartDashboard.putString("Current command", "SetInputLambda");
    }

    protected void execute() {
        double currentTime = this.m_timer.get();
        Robot.motor.setInput(this.m_inputCalculator.operation(currentTime));
    }

    @Override
    protected boolean isFinished() {
        return false;
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