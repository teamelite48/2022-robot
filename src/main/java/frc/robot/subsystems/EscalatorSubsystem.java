// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.PwmPort;

public class EscalatorSubsystem extends SubsystemBase {
  
  private final PWMSparkMax motor = new PWMSparkMax(PwmPort.EscalatorMotor);

  private final double motorSpeed = 0.5;

  public EscalatorSubsystem() {}

  @Override
  public void periodic() {
    
  }
  public void forward(){
    motor.set(motorSpeed);
  }
public void backwards(){
  motor.set(-motorSpeed);
}

}
