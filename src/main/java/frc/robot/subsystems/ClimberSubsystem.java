// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CanBusId;
import frc.robot.config.PneumaticChannel;

public class ClimberSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftArmMotor = new WPI_TalonFX(CanBusId.LeftClimberMotor);
  private final WPI_TalonFX rightArmMotor = new WPI_TalonFX(CanBusId.RightClimberMotor);

  private final DoubleSolenoid leftArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftArmForward, PneumaticChannel.LeftArmReverse);
  private final DoubleSolenoid rightArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightArmForward, PneumaticChannel.RightArmReverse);

  private final double motorSpeed = 0.5;
  private boolean isClimberEnabled = false;

  public ClimberSubsystem() {
    leftArmSolenoid.set(Value.kReverse);
    rightArmSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climber Enabled", isClimberEnabled);
    SmartDashboard.putString("Left Arm Position", leftArmSolenoid.get().toString());
    SmartDashboard.putString("Right Arm Position", rightArmSolenoid.get().toString());
  }

  public void enableClimber() {
    isClimberEnabled = true;
  }

  public void toggleLeftArmPosition() {
    if (isClimberEnabled == false){
      return;
    }

    leftArmSolenoid.toggle();
  }

  public void toggleRightArmPosition() {
    if (isClimberEnabled == false){
      return;
    }
    
    rightArmSolenoid.toggle();
  }

  public void adjustArmLengths(double leftSpeed, double rightSpeed) {
    if (isClimberEnabled == false) {
      return;
    }

    leftArmPosition(leftSpeed);
    rightArmPosition(rightSpeed);
  }

  public void leftArmPosition(double speed) {
    leftArmMotor.set(speed);
  }

  public void rightArmPosition(double speed){
    rightArmMotor.set(speed);
  }
}
