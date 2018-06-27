package frc.team687.robot;


import edu.wpi.first.wpilibj.TimedRobot;
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
    public void teleopInit() { }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() { }

    @Override
    public void testPeriodic() { }
}