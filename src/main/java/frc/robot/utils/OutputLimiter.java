package frc.robot.utils;

public class OutputLimiter {

    double lowerLimit;
    double upperLimit;

    public OutputLimiter(double lowerLimit, double upperLimit) {
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
    }

    public double limit(double value) {

        if (value <= lowerLimit) {
            return lowerLimit;
        }

        else if (value >= upperLimit) {
            return upperLimit;
        }

        return value;
    }
}
