package frc.robot.config.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class ClimberConfig {

    public static final double extendArmSpeed = 0.5;
    public static final double retractArmSpeed = -extendArmSpeed;

    public static Value initialArmPosition = Value.kForward;

    public static final int armExtensionLimit = 100;
    public static final int armRetractionLimit = 10;
}
