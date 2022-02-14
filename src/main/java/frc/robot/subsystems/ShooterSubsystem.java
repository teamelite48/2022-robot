// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX topMotor= new WPI_TalonFX(CanBusId.TopShooterMotor);
  private final WPI_TalonFX bottomMotor = new WPI_TalonFX(CanBusId.BottomShooterMotor);

  public ShooterSubsystem() {

    topMotor.configFactoryDefault();
    bottomMotor.configFactoryDefault();

    bottomMotor.follow(topMotor);
  }

  @Override
  public void periodic() {}

  public void shoot() {
    topMotor.set(1.0);
  }
}
