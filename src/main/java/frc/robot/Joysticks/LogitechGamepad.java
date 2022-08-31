package frc.robot.Joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechGamepad {

    public JoystickButton lb;
    public JoystickButton rb;

    public Trigger up;
    public Trigger down;
    public Trigger left;
    public Trigger right;

    public JoystickButton x;
    public JoystickButton a;
    public JoystickButton y;
    public JoystickButton b;

    public JoystickButton start;
    public JoystickButton back;

    private GenericHID hid;

    public LogitechGamepad(int port) {
        hid = new GenericHID(port);

        lb = new JoystickButton(hid, 5);
        rb = new JoystickButton(hid, 6);

        start = new JoystickButton(hid, 8);
        back = new JoystickButton(hid, 7);

        x = new JoystickButton(hid, 3);
        a = new JoystickButton(hid, 1);
        y = new JoystickButton(hid, 4);
        b = new JoystickButton(hid, 2);
    }

    public double getLeftXAxis() {
        return modifyAxis(hid.getRawAxis(0));
    }
    public double getRightXAxis(){
        return modifyAxis(hid.getRawAxis(4));
    }

    public double getLeftYAxis() {
        return modifyAxis(hid.getRawAxis(1));
    }

    public double getRightYAxis(){
        return modifyAxis(hid.getRawAxis(5));
    }

    private double modifyAxis(double input) {

        if (-0.5 <= input && input <= 0.5) return 0;

        return input * Math.abs(input);
      }
}
