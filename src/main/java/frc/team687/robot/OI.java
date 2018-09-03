package frc.team687.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team687.robot.commands.ResetMotorEncoder;
import frc.team687.robot.commands.TrackReference;
import frc.team687.robot.constants.TestMotorConstants;

public class OI {

    public Joystick testStick = new Joystick(0);

    public JoystickButton testButton_3;

    public OI() {
        testButton_3 = new JoystickButton(testStick, 3);
        testButton_3.whenPressed(new TrackReference(TestMotorConstants.kDefaultGoal));

        SmartDashboard.putData("Testy pls", new TrackReference(TestMotorConstants.kDefaultGoal));
        SmartDashboard.putData("Reset pls", new ResetMotorEncoder());
    }
}
