package frc.robot.config.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class ClimberConfig {

    public static final double extendArmSpeed = 0.5;
    public static final double retractArmSpeed = -1.0;

    public static Value upTilt = Value.kForward;
    public static Value downTilt = Value.kReverse;
    public static Value initialArmPosition = downTilt;

    public static Value lockValue = Value.kReverse;
    public static Value unlockValue = Value.kForward;

    public static final int downTiltArmExtensionLimit = 180000;
    public static final int downTiltArmRetractionLimit = 2048;

    public static final int upTiltArmExtensionLimit = 125000;
    public static final int upTiltArmRetractionLimit = 2048;

    //tilt down extension limit = 180000
    //tilt up extension limit = 125000
    
}
