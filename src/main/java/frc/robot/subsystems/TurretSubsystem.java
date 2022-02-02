// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.TurretConfig;
import frc.robot.config.roborio.CanBusId;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CanBusId.TurretMotor, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  public TurretSubsystem() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Postiton", encoder.getPosition());
  }

  public void simulationPeriodic() {
    // int encoderTicksPerPeriodic = 100;
    // int newPosition = (int) (encoder.getPosition() + (motor.getAppliedOutput() * encoderTicksPerPeriodic));
    // encoder.setPosition(newPosition);
  }

  public void rotateClockwise() {
    motor.set(TurretConfig.motorSpeed);
  }

  public void rotateCounterClockwise() {
    motor.set(-TurretConfig.motorSpeed);
  }

  public void stop() {
    motor.set(0);
  }

  // method to auto aim
  // if the target is in view than try to lock on
  // calcluate new speed
  // adjust motor if the desired speed won't turn the turret outside of it's allowed range
}
