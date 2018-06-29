package frc.team687.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team687.robot.commands.ResetMotorEncoder;
import frc.team687.robot.subsystems.TestMotor;

public class Robot extends TimedRobot {

    public static TestMotor motor;

    @Override
    public void robotInit() {
        motor = new TestMotor();
    }

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
        Scheduler.getInstance().add(new ResetMotorEncoder());
    }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() { }
}