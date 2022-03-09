package frc.robot.config.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class ClimberConfig {

    public static final double extendArmSpeed = 0.5;
    public static final double retractArmSpeed = -extendArmSpeed;
    public static final double slowRetractArmSpeed = 0.15;


    public static Value upTilt = Value.kForward;
    public static Value downTilt = Value.kReverse;
    public static Value initialArmPosition = downTilt;

    public static Value lockValue = Value.kReverse;
    public static Value unlockValue = Value.kForward;

    public static final int armExtensionLimit = 180000;
    public static final int armRetractionLimit = 0;

    //tilt down extension limit = 180000
    //tilt up extension limit = 125000
    
}
