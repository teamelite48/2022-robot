package frc.robot.Joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS4Gamepad extends GenericHID {

    public PS4Gamepad(int port) {
        super(port);
    }

    public double getLeftX() { return this.getRawAxis(0); }
    public double getLeftY() { return this.getRawAxis(1); }
    public double getRightX() { return this.getRawAxis(2); }
    public double getRightY() { return this.getRawAxis(5); }

    public JoystickButton getSquareButton() { return new JoystickButton(this, 1); }
    public JoystickButton getCrossButton() { return new JoystickButton(this, 2); }
    public JoystickButton getCircleButton() { return new JoystickButton(this, 3); }
    public JoystickButton getTriangleButton() { return new JoystickButton(this, 4); }

    public JoystickButton getL1Button() { return new JoystickButton(this, 5); }
    public JoystickButton getL2Button() { return new JoystickButton(this, 7); }

    public JoystickButton getR1Button() { return new JoystickButton(this, 6); }
    public JoystickButton getR2Button() { return new JoystickButton(this, 8); }

    public JoystickButton getBackButton() { return new JoystickButton(this, 9); }
    public JoystickButton getStartButton() { return new JoystickButton(this, 10); }

    public JoystickButton getLeftStickButton() { return new JoystickButton(this, 11); }
    public JoystickButton getRightStickButton() { return new JoystickButton(this, 12); }

    public JoystickButton getPSButton() { return new JoystickButton(this, 13); }
    public JoystickButton getTouchpadButton() { return new JoystickButton(this, 14); }

    public Trigger getDpadUpTrigger() { return new Trigger(() -> this.getPOV() == 0); }
    public Trigger getDpadRightTrigger() { return new Trigger(() -> this.getPOV() == 90); }
    public Trigger getDpadDownTrigger() { return new Trigger(() -> this.getPOV() == 180); }
    public Trigger getDpadLeftTrigger() { return new Trigger(() -> this.getPOV() == 270); }

}
