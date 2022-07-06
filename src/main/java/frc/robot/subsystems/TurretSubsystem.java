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
import frc.robot.utils.OutputLimiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurretSubsystem extends SubsystemBase {

  final CANSparkMax motor = new CANSparkMax(CanBusId.TurretMotor, MotorType.kBrushless);
  final RelativeEncoder encoder = motor.getEncoder();
  final PIDController pidController = new PIDController(TurretConfig.kP, TurretConfig.kI, TurretConfig.kD);
  final OutputLimiter motorOutputLimiter = new OutputLimiter(-TurretConfig.motorMaxOutput, TurretConfig.motorMaxOutput);

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  final NetworkTableEntry tx = table.getEntry("tx");
  final NetworkTableEntry tv = table.getEntry("tv");
  final NetworkTableEntry ledMode = table.getEntry("ledMode");
  final NetworkTableEntry camMode = table.getEntry("camMode");

  boolean isAutoAimEnabled = true;
  boolean isAutoAimOn = false;

  long lastSimulationPeriodicMillis = 0;
  boolean isTurretEnabled = true;

  public TurretSubsystem() {
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, TurretConfig.encoderLimit);
    motor.setSoftLimit(SoftLimitDirection.kReverse, -TurretConfig.encoderLimit);

    ledMode.setNumber(3);
    camMode.setNumber(1);
  }

  @Override
  public void periodic() {

    if (isAutoAimOn) {
      ledMode.setNumber(3);
      camMode.setNumber(0);
      autoAim();
    }
    else {
      ledMode.setNumber(1);
      camMode.setNumber(1);
    }

    SmartDashboard.putNumber("Turret Degrees", getPositionInDegrees());
    SmartDashboard.putNumber("Turret tx", tx.getDouble(0.0));
    SmartDashboard.putBoolean("Auto Aim Enabled", isAutoAimEnabled);
    SmartDashboard.putBoolean("Auto Aim On", isAutoAimOn);
    SmartDashboard.putBoolean("Target Acquired", isTargetAcquired());
  }

  public void simulationPeriodic() {

    long millisSinceLastPeriodic = System.currentTimeMillis() - lastSimulationPeriodicMillis;
    double elapsedSeconds = (millisSinceLastPeriodic / 1000.0);
    double rotationsSinceLastPeriodic = motor.get() * elapsedSeconds * TurretConfig.nominalMotorRotationsPerSecond;

    encoder.setPosition(encoder.getPosition() + rotationsSinceLastPeriodic);

    lastSimulationPeriodicMillis = System.currentTimeMillis();
  }

  public void enableTurret() {
    isTurretEnabled = true;
  }

  public void disableTurret() {
    isTurretEnabled = false;
    turnAutoAimOff();
  }

  public void enableAutoAim() {
    isAutoAimEnabled = true;
  }

  public void disableAutoAim() {
    isAutoAimEnabled = false;
    isAutoAimOn = false;
  }

  public void turnAutoAimOn() {
    if (isAutoAimEnabled == false) return;

    isAutoAimOn = true;
  }

  public void turnAutoAimOff() {
    isAutoAimOn = false;
  }

  public void autoAim() {

    if(isTargetAcquired() == false) return;

    double error = tx.getDouble(0.0);
    double newMotorSpeed = motorOutputLimiter.limit(error * TurretConfig.kP);

    setMotor(newMotorSpeed);
  }

  public boolean isTargetAcquired(){
    return tv.getDouble(0) == 1 ? true: false;
  }

  public void stop() {
    motor.set(0);
  }

  public void moveToDegrees(Double degrees) {
    double motorSpeed = pidController.calculate(getPositionInDegrees(), degrees);
    double limitedMotorSpeed = motorOutputLimiter.limit(motorSpeed);

    setMotor(limitedMotorSpeed);
  }

  public void manualTurret(double leftX) {

    if (Math.abs(leftX) >= TurretConfig.inputDeadzone) {
      turnAutoAimOff();

      double scaledInput = leftX * Math.abs(leftX) * TurretConfig.motorMaxOutput;

      setMotor(scaledInput);
    }
    else if (isAutoAimOn == false) {
      stop();
    }
  }

  public void rotateClockwise() {
    turnAutoAimOff();
    setMotor(TurretConfig.clockwiseSpeed);
  }

  public void rotateCounterClockwise() {
    turnAutoAimOff();
    setMotor(TurretConfig.counterClockwiseSpeed);
  }

  public double getPositionInDegrees() {
    return encoder.getPosition() * TurretConfig.degreesPerMotorRotation + TurretConfig.degreesAtCenter;
  }

  public void setMotor(double speed) {
    if (isTurretEnabled == false) {
      stop();
      return;
    }

    motor.set(speed);
  }
}
