// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.subsystems.ElevatorConfig;

public class ShooterFeedSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CanBusId.ShooterFeedMotor, MotorType.kBrushless);

  public ShooterFeedSubsystem() {}

  @Override
  public void periodic() {

  }

  public void up(){
    motor.set(ElevatorConfig.motorSpeed);
  }

  public void down(){
    motor.set(-ElevatorConfig.motorSpeed);
  }

  public void stop(){
    motor.set(0);

  }



}
