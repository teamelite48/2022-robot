// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final PWMSparkMax motor = new PWMSparkMax(0);
    
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    
  }
  public void intake() {
    motor.set(0.5);
  }
  
  public void outtake() {
    motor.set(-0.5);
  }

  public void stop() {
      motor.set(0);
  }
}
