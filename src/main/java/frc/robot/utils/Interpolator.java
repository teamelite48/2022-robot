package frc.robot.utils;

import java.util.HashMap;

public class Interpolator {

    HashMap<Integer, Integer> domainAndRange;

    public Interpolator(HashMap<Integer, Integer> domainAndRange) {
        this.domainAndRange = domainAndRange;
    }

    public double calculate(double x) {

        double x1 = (int) x;
        double x2 = (int) (x + 1);
        double y1 = domainAndRange.getOrDefault(x1, 0);
        double y2 = domainAndRange.getOrDefault(x2, 0);

        double y = y1 + (x - x1) * (y2 - y1);

        return y;
    }
}
