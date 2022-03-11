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
import frc.robot.commands.turret.DisableAutoAim;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.subsystems.ShooterConfig;

public class ShooterSubsystem extends SubsystemBase {

  final WPI_TalonFX leftMotor = new WPI_TalonFX(CanBusId.LeftShooterMotor);
  final WPI_TalonFX rightMotor = new WPI_TalonFX(CanBusId.RightShooterMotor);

  final Solenoid deflectorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.Deflector);
  final PIDController pidController = new PIDController(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD);

  boolean isShooterOn = false;
  double targetRPM = ShooterConfig.mediumRPM;


  public ShooterSubsystem() {

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    rightMotor.setInverted(true);
    leftMotor.follow(rightMotor);
    leftMotor.setInverted(InvertType.OpposeMaster);

    SmartDashboard.putNumber("Current RPM", 0);
    SmartDashboard.putNumber("PIDValue", 0);
  }

  @Override
  public void periodic() {

    if (isShooterOn == false) {
      rightMotor.set(0);
      
    }
    else {
      double currentRPM = -1 * (rightMotor.getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048;
      SmartDashboard.putNumber("Current RPM", currentRPM);

      double motorOutput = targetRPM/ShooterConfig.maxRPM + pidController.calculate(currentRPM, targetRPM);
      SmartDashboard.putNumber("PIDValue", pidController.calculate(currentRPM, targetRPM));

      rightMotor.set(Math.max(motorOutput, 0));
    }

    SmartDashboard.putNumber("Target RPM", targetRPM);
    SmartDashboard.putBoolean("Shooter On", isShooterOn);
    SmartDashboard.putString("Deflector Position", deflectorSolenoid.get() ? "Forward" : "Backward");
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
    targetRPM = ShooterConfig.lowRPM;
    moveDeflectorBackward();
  }

  public void setMediumSpeed() {
    targetRPM = ShooterConfig.mediumRPM;
    moveDeflectorForward();
  }

  public void setHighSpeed() {
    targetRPM = ShooterConfig.highRPM;
    moveDeflectorForward();
  }

  private void moveDeflectorForward() {
    deflectorSolenoid.set(ShooterConfig.deflectorForwardValue);
  }

  private void moveDeflectorBackward() {
    deflectorSolenoid.set(ShooterConfig.deflectorBackwardValue);
  }

  public boolean isShooterOn() {
      return isShooterOn;
  }

  public void bumpRpmUp() {
    targetRPM += ShooterConfig.rpmBump;
  }

  public void bumpRpmDown() {
    targetRPM -= ShooterConfig.rpmBump;
  }
}
