package frc.team687.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team687.robot.commands.ResetMotorEncoder;
import frc.team687.robot.subsystems.TestMotor;
import frc.team687.robot.OI;

public class Robot extends TimedRobot {

    public static TestMotor motor;
    public static OI oi;

    @Override
    public void robotInit() {
        motor = new TestMotor();

        oi = new OI();
    }

    @Override
    public void disabledInit() {
        // motor.startLog();
        motor.stopLog();
    }

    @Override
    public void autonomousInit() {
        motor.startLog();
     }

    @Override
    public void teleopInit() {
        Scheduler.getInstance().add(new ResetMotorEncoder());
        
        motor.startLog();
    }

    @Override
    public void testInit() {
        motor.startLog();
    }


    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().removeAll();

        motor.reportToSmartDashboard();
    }
    
    @Override
    public void autonomousPeriodic() {
        motor.reportToSmartDashboard();
        motor.logToCSV();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        motor.reportToSmartDashboard();
        motor.logToCSV();
    }

    @Override
    public void testPeriodic() {
        motor.reportToSmartDashboard();
        motor.logToCSV();
    }
}