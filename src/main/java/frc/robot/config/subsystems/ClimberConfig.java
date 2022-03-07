package frc.robot.config.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class ClimberConfig {

    public static final double extendArmSpeed = 0.3;
    public static final double retractArmSpeed = -extendArmSpeed;

    public static Value upTilt = Value.kForward;
    public static Value downTilt = Value.kReverse;
    public static Value initialArmPosition = downTilt;

    public static Value lockValue = Value.kForward;
    public static Value unlockValue = Value.kReverse;

    public static final int armExtensionLimit = 20;
    public static final int armRetractionLimit = 5;
}
