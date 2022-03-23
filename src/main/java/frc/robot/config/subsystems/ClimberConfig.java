package frc.robot.config.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class ClimberConfig {

    public static final double extendArmSpeed = 0.8;
    public static final double retractArmSpeed = -1.0;

    public static Value upTilt = Value.kForward;
    public static Value downTilt = Value.kReverse;

    public static Value lockValue = Value.kReverse;
    public static Value unlockValue = Value.kForward;

    public static final int downTiltArmExtensionLimit = 225000;
    public static final int downTiltArmRetractionLimit = 4096;

    public static final int upTiltArmExtensionLimit = 125000;
    public static final int upTiltArmRetractionLimit = 4096;

    public static final double armSpeedDeadband = 0.02;
}
