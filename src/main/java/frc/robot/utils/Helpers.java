package frc.robot.utils;

public class Helpers {
    public static double limit(double input, double limit) {

        if (input <= -limit) {
            return -limit;
        }

        else if (input >= limit) {
            return limit;
        }

        return input;
    }
}
