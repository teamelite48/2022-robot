package frc.robot.config.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class IntakeConfig {

    public static final double intakeSpeed = 1;
    public static final double outtakeSpeed = -intakeSpeed;

    public static final Value deployValue = Value.kForward;
    public static final Value retractValue = Value.kReverse;
}
