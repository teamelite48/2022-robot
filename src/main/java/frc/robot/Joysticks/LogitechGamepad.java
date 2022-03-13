package frc.robot.Joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechGamepad extends GenericHID {

    public LogitechGamepad(int port) {
        super(port);
    }

    public double getLeftX() { return this.getRawAxis(0); }
    public double getLeftY() { return this.getRawAxis(1); }
    public double getRightX() { return this.getRawAxis(4); }
    public double getRightY() { return this.getRawAxis(5); }

    public JoystickButton getXButton() { return new JoystickButton(this, 1); }
    public JoystickButton getAButton() { return new JoystickButton(this, 2); }
    public JoystickButton getBButton() { return new JoystickButton(this, 3); }
    public JoystickButton getYButton() { return new JoystickButton(this, 4); }

    public JoystickButton getLeftBumper() { return new JoystickButton(this, 5); }
    public JoystickButton getLeftTrigger() { return new JoystickButton(this, 7); }

    public JoystickButton getRightBumper() { return new JoystickButton(this, 6); }
    public JoystickButton getRightTrigger() { return new JoystickButton(this, 8); }

    public JoystickButton getBackButton() { return new JoystickButton(this, 9); }
    public JoystickButton getStartButton() { return new JoystickButton(this, 10); }

    public JoystickButton getLeftStickButton() { return new JoystickButton(this, 11); }
    public JoystickButton getRightStickButton() { return new JoystickButton(this, 12); }


    public Trigger getDpadUpTrigger() { return new Trigger(() -> this.getPOV() == 0); }
    public Trigger getDpadRightTrigger() { return new Trigger(() -> this.getPOV() == 90); }
    public Trigger getDpadDownTrigger() { return new Trigger(() -> this.getPOV() == 180); }
    public Trigger getDpadLeftTrigger() { return new Trigger(() -> this.getPOV() == 270); }
}
