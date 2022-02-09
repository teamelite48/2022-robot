// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.EscalatorConfig;
import frc.robot.config.PwmPort;

public class SorterSubsystem extends SubsystemBase {

  private final PWMSparkMax motor = new PWMSparkMax(PwmPort.SorterMotor);

  public SorterSubsystem() {}

  @Override
  public void periodic() {

  }

  public void in(){
    motor.set(EscalatorConfig.MotorSpeed);
  }

  public void out(){
    motor.set(-EscalatorConfig.MotorSpeed);
  }

  public void stop(){
    motor.set(0);
  }
}
