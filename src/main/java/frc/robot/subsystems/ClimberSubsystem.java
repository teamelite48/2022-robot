// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CanBusId;
import frc.robot.config.PneumaticChannel;

public class ClimberSubsystem extends SubsystemBase {

  //TODO: Create Solenoids x 2
  //TODO: Create 2 WPILib Controllers
  
  private final WPI_TalonFX leftArmMotor = new WPI_TalonFX(CanBusId.LeftClimberMotor);

  private final DoubleSolenoid leftArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftArmForward, PneumaticChannel.LeftArmReverse);

  private final double motorSpeed = 0.5;

  public ClimberSubsystem() {
    //TODO: Set default pos to back
    leftArmSolenoid.set(Value.kReverse);

  }

  @Override
  public void periodic() {}

  //TODO: Method to extend/retract right/left arms
  //TODO: Method to toggle forward/back independently

  public void toggleLeftArm() {
    leftArmSolenoid.toggle();
  }

  public void extendLeftArm() {
    leftArmMotor.set(motorSpeed);
  }
}
