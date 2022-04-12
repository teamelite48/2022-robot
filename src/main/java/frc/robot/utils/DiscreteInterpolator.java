package frc.robot.utils;

import java.util.HashMap;

public class DiscreteInterpolator {

    HashMap<Integer, Integer> domainAndRange;

    public DiscreteInterpolator(HashMap<Integer, Integer> domainAndRange) {
        this.domainAndRange = domainAndRange;
    }

    public int interpolate(double x) {

        double x1 = (int) x;
        double x2 = (int) (x + 1);
        double y1 = domainAndRange.getOrDefault(x1, 0);
        double y2 = domainAndRange.getOrDefault(x2, 0);

        double y = y1 + (x - x1) * (y2 - y1);

        return (int) y;
    }
}
