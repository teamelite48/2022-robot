// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CanBusId;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX motor= new WPI_TalonFX(CanBusId.ShooterMotor);

  public ShooterSubsystem() {}

  @Override
  public void periodic() {}

  public void shoot() {
    motor.set(1.0);
  }
}
