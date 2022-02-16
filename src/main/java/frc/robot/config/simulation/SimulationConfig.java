package frc.robot.config.simulation;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;

public final class SimulationConfig {
    // TODO: Create feedforward gain constants (from the characterization tool)
    public static final double kvLinear = 1.98;
    public static final double kaLinear = 0.2;
    public static final double kvAngular = 1.5;
    public static final double kaAngular = 0.3;

    // TODO: Get real values
    public static final double gearingReduction = 7.29;

    public static final Vector<N7> EnocoderNoise = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
}
