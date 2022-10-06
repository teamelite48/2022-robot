package frc.robot.Joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechGamepad {

    public JoystickButton lb;
    public JoystickButton rb;

    public Trigger lt;
    public Trigger rt;

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

    private double deadband = 0.02;

    public LogitechGamepad(int port) {
        hid = new GenericHID(port);

        lb = new JoystickButton(hid, 5);
        rb = new JoystickButton(hid, 6);

        lt = new Trigger(() -> hid.getRawAxis(2) > this.deadband);
        rt = new Trigger(() -> hid.getRawAxis(3) > this.deadband);

        up = new Trigger(() -> hid.getPOV() == 0);
        right = new Trigger(() -> hid.getPOV() == 90);
        down = new Trigger(() -> hid.getPOV() == 180);
        left = new Trigger(() -> hid.getPOV() == 270);

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
        if ((-deadband <= input && input <= 0.0) || (0.0 <= input && input <= deadband)) return 0;

        return input * Math.abs(input) * Math.abs(input) / 3.0;
      }
}
