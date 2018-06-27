package frc.team687.robot.commands;

import Jama.Matrix;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.team687.robot.Robot;
import frc.team687.robot.subsystems.TestMotor;

public class TrackReference extends Command {

    private Matrix m_reference;

    private Notifier m_ssRunner;

    public TrackReference(Matrix reference) {
        requires(Robot.motor);
        this.m_reference = reference;

        this.m_ssRunner = new Notifier( () -> {
            Robot.motor.trackGoal();
        });
    }

    protected void initialize() {
        Robot.motor.setGoal(this.m_reference);
        m_ssRunner.startPeriodic(.01);
    }

    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.motor.setGoal(TestMotor.kEquilibriumGoal);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        this.end();
    }
}
