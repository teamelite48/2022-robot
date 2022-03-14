package frc.robot.Joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class LogitechJoystick extends GenericHID {

    public LogitechJoystick(int port) {
        super(port);
    }

    public double getX() { return getRawAxis(0); }
    public double getY() { return getRawAxis(1); }
    public double getZ() { return getRawAxis(2); }

    public JoystickButton getTrigger() { return new JoystickButton(this, 1); }
    public JoystickButton getButton2() { return new JoystickButton(this, 2); }
    public JoystickButton getButton4() { return new JoystickButton(this, 4); }
    public JoystickButton getButton5() { return new JoystickButton(this, 5); }
    public JoystickButton getButton8() { return new JoystickButton(this, 8); }
    public JoystickButton getButton9() { return new JoystickButton(this, 9); }

}
