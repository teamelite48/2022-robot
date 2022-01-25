// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.PneumaticChannel;
import frc.robot.config.PwmPort;

public class IntakeSubsystem extends SubsystemBase {
  
  private final PWMSparkMax motor = new PWMSparkMax(PwmPort.IntakeMotor);
  private final Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH,PneumaticChannel.IntakeForward);
  private boolean isIntakeDeployed = false;
  
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    
  }
  
  public void intake() {
    if (isIntakeDeployed == false) {
      deploy();
    }

    motor.set(0.5);
  }
  
  public void outtake() {
    motor.set(-0.5);
  }

  public void stop() {
      motor.set(0);
  }

  public void deploy(){
    intakeSolenoid.set(true);
    isIntakeDeployed = true; 
  }

  public void retract(){
    intakeSolenoid.set(false);
    isIntakeDeployed = false;
  }
}
