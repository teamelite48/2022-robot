// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.subsystems.TurretConfig;
import frc.robot.utils.Helpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurretSubsystem extends SubsystemBase {

  final CANSparkMax motor = new CANSparkMax(CanBusId.TurretMotor, MotorType.kBrushless);
  final RelativeEncoder encoder = motor.getEncoder();
  final PIDController pidController = new PIDController(TurretConfig.kP, TurretConfig.kI, TurretConfig.kD);

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  final NetworkTableEntry tx = table.getEntry("tx");
  final NetworkTableEntry tv = table.getEntry("tv");
  final NetworkTableEntry ledMode = table.getEntry("ledMode");

  boolean isAutoAimEnabled = false;
  long lastSimulationPeriodicMillis = 0;

  public TurretSubsystem() {
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, TurretConfig.encoderLimit);
    motor.setSoftLimit(SoftLimitDirection.kReverse, -TurretConfig.encoderLimit);
  }

  @Override
  public void periodic() {

    if (isAutoAimEnabled) {
      ledMode.setNumber(3);
      autoAim();
    }
    else {
      ledMode.setNumber(1);
    }


    SmartDashboard.putNumber("Turret Degrees", getPositionInDegrees());
    SmartDashboard.putNumber("Turret tx", tx.getDouble(0.0));
    SmartDashboard.putBoolean("Auto Aim", isAutoAimEnabled);
  }

  public void simulationPeriodic() {

    long millisSinceLastPeriodic = System.currentTimeMillis() - lastSimulationPeriodicMillis;
    double elapsedSeconds = (millisSinceLastPeriodic / 1000.0);
    double rotationsSinceLastPeriodic = motor.get() * elapsedSeconds * TurretConfig.nominalMotorRotationsPerSecond;

    encoder.setPosition(encoder.getPosition() + rotationsSinceLastPeriodic);

    lastSimulationPeriodicMillis = System.currentTimeMillis();
  }

  public void enableAutoAim() {
    isAutoAimEnabled = true;
  }

  public void disableAutoAim() {
    isAutoAimEnabled = false;
  }

  public void autoAim() {

    // TODO: verify encoder and motor movements
    // if (RobotBase.isReal()) return;

    double error = tx.getDouble(0.0);
    boolean targetAcquired = tv.getBoolean(false);

    if(targetAcquired == false) return;

    double newMotorSpeed = Helpers.limit(error * TurretConfig.kP, TurretConfig.motorMaxOutput);

    motor.set(newMotorSpeed);
  }

  public void stop() {
    motor.set(0);
  }

  public void moveToDegrees(Double degrees) {
    double motorSpeed = pidController.calculate(getPositionInDegrees(), degrees);
    double limitedMotorSpeed = Helpers.limit(motorSpeed, TurretConfig.motorMaxOutput);

    motor.set(limitedMotorSpeed);
  }

  public void manualTurret(double leftX) {

    if (Math.abs(leftX) >= TurretConfig.inputDeadzone) {
      disableAutoAim();

      double scaledInput = leftX * Math.abs(leftX) * TurretConfig.motorMaxOutput;

      motor.set(scaledInput);
    }
    else if (isAutoAimEnabled == false) {
      stop();
    }
  }

  public double getPositionInDegrees() {
    return encoder.getPosition() * TurretConfig.degreesPerMotorRotation + TurretConfig.degreesAtCenter;
  }
}
