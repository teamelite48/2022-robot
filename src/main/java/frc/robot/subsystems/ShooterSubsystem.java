// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.subsystems.ShooterConfig;

public class ShooterSubsystem extends SubsystemBase {

  final WPI_TalonFX leftMotor = new WPI_TalonFX(CanBusId.LeftShooterMotor);
  final WPI_TalonFX rightMotor = new WPI_TalonFX(CanBusId.RightShooterMotor);

  final Solenoid deflectorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.Deflector);
  final PIDController pidController = new PIDController(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD);

  boolean isShooterOn = false;
  double targetOuput = ShooterConfig.lowSpeed;


  public ShooterSubsystem() {

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    rightMotor.setInverted(true);
    leftMotor.follow(rightMotor);
    leftMotor.setInverted(InvertType.OpposeMaster);
  }

  @Override
  public void periodic() {

    if (isShooterOn == false) {
      leftMotor.set(0);
    }
    else {
      // TODO: ask Travis to blar
      double currentOutput = leftMotor.getMotorOutputPercent();

      double output = pidController.calculate(currentOutput, targetOuput);

      leftMotor.set(output);
    }

    SmartDashboard.putNumber("Target Speed", targetOuput);
    SmartDashboard.putBoolean("Shooter On", isShooterOn);

    SmartDashboard.putString("Left Deflector", deflectorSolenoid.get() ? "Forward" : "Backward");
  }

  public void toggleShooter() {
    isShooterOn = !isShooterOn;
  }

  public void shooterOn() {
    isShooterOn = true;
  }

  public void shooterOff() {
    isShooterOn = false;
  }

  public void setLowSpeed() {
    targetOuput = ShooterConfig.lowSpeed;
    moveDeflectorForward();
  }

  public void setMediumSpeed() {
    targetOuput = ShooterConfig.mediumSpeed;
    moveDeflectorBackward();
  }

  public void setHighSpeed() {
    targetOuput = ShooterConfig.highSpeed;
    moveDeflectorBackward();
  }

  private void moveDeflectorForward() {
    deflectorSolenoid.set(ShooterConfig.deflectorForwardValue);
  }

  private void moveDeflectorBackward() {
    deflectorSolenoid.set(ShooterConfig.deflectorBackwardValue);
  }
}
