// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.PwmPort;

public class ElevatorSubsystem extends SubsystemBase {

  private final PWMSparkMax motor = new PWMSparkMax(PwmPort.ElevatorMotor);
 
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    
  }
  
  public void up(){
    motor.set(ElevatorConfig.MotorSpeed);
  }

  public void down(){
    motor.set(-ElevatorConfig.MotorSpeed);
  }

  public void stop(){
    motor.set(0);

  }



}
