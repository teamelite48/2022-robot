// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.subsystems.ShooterConfig;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX topMotor = new WPI_TalonFX(CanBusId.TopShooterMotor);
  private final WPI_TalonFX bottomMotor = new WPI_TalonFX(CanBusId.BottomShooterMotor);

  private final Solenoid deflectorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.Deflector);

  private boolean isShooterOn = false;
  private double targetSpeed = ShooterConfig.lowSpeed;

  public ShooterSubsystem() {

    topMotor.configFactoryDefault();
    bottomMotor.configFactoryDefault();

    bottomMotor.follow(topMotor);
  }

  @Override
  public void periodic() {

    if (isShooterOn == false) {
      topMotor.set(0);
    }
    else {
      topMotor.set(targetSpeed);
    }

    SmartDashboard.putNumber("Target Speed", targetSpeed);
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
    targetSpeed = ShooterConfig.lowSpeed;
    moveDeflectorForward();
  }

  public void setMediumSpeed() {
    targetSpeed = ShooterConfig.mediumSpeed;
    moveDeflectorBackward();
  }

  public void setHighSpeed() {
    targetSpeed = ShooterConfig.highSpeed;
    moveDeflectorBackward();
  }

  private void moveDeflectorForward() {
    deflectorSolenoid.set(ShooterConfig.deflectorForwardValue);
  }

  private void moveDeflectorBackward() {
    deflectorSolenoid.set(ShooterConfig.deflectorBackwardValue);
  }
}
