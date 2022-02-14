package frc.robot.config.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class ClimberConfig {
    public static final double motorSpeed = 0.5;
    public static final int forwardLimit = 100;
    public static final int reverseLimit = 10;
    public static Value initialArmPosition = Value.kForward;
}
