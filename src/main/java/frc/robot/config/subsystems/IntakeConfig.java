package frc.robot.config.subsystems;

public final class IntakeConfig {

    public static final double intakeSpeed = 1;
    public static final double outtakeSpeed = -intakeSpeed;

    public static final boolean retractValue = false;
    public static final boolean deployValue = !retractValue;
    
    public static final long deployIntakeCooldown = 300;
}
