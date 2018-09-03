package frc.team687.robot.commands;

import Jama.Matrix;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team687.robot.Robot;
import frc.team687.robot.constants.TestMotorConstants;

public class TrackReference extends Command {

    private Matrix m_reference;

//    private Notifier m_ssRunner;

    public TrackReference(Matrix reference) {
        requires(Robot.motor);
        this.m_reference = reference;

//        this.m_ssRunner = new Notifier( () -> {
//
//        } );
    }

    protected void initialize() {
        Robot.motor.setGoal(this.m_reference);
        SmartDashboard.putString("Current command", "Track Reference");
//        m_ssRunner.startPeriodic(Robot.motor.gains().dt);
    }

    protected void execute() {
        Robot.motor.trackGoal();
        // Robot.motor.setVoltage(this.m_reference.get(0,0));
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
