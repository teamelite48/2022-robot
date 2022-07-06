package frc.robot.utils;

import java.util.HashMap;

public class LinearInterpolator {

    HashMap<Integer, Integer> domainAndRange;

    public LinearInterpolator(HashMap<Integer, Integer> domainAndRange) {
        this.domainAndRange = domainAndRange;
    }

    public double calculate(double x) {

        int x1 = (int) Math.floor(x);
        int x2 = (int) Math.ceil(x);
        double y1 = domainAndRange.getOrDefault(x1, 0);
        double y2 = domainAndRange.getOrDefault(x2, 0);

        double y = y1 + (x - x1) * (y2 - y1);

        return y;
    }
}
