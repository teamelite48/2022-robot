// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.subsystems.TurretConfig;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CanBusId.TurretMotor, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  long lastSimulationPeriodicMillis = System.currentTimeMillis();


  public TurretSubsystem() {

    // motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // motor.setSoftLimit(SoftLimitDirection.kForward, TurretConfig.encoderLimit);
    // motor.setSoftLimit(SoftLimitDirection.kReverse, -TurretConfig.encoderLimit);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Postiton", encoder.getPosition());
  }

  public void simulationPeriodic() {

    long millisSinceLastPeriodic = System.currentTimeMillis() - lastSimulationPeriodicMillis;
    double elapsedSeconds = (millisSinceLastPeriodic / 1000.0);
    double rotationsSinceLastPeriodic = motor.get() * elapsedSeconds * TurretConfig.nominalMotorRotationsPerSecond;

    encoder.setPosition(encoder.getPosition() + rotationsSinceLastPeriodic);

    lastSimulationPeriodicMillis = System.currentTimeMillis();
  }

  public void rotateClockwise() {

    if(encoder.getPosition() >= TurretConfig.encoderLimit){
      motor.set(0);
    }
    else {
      motor.set(TurretConfig.clockwiseSpeed);
    }
  }

  public void rotateCounterClockwise() {
    if(encoder.getPosition() <= -TurretConfig.encoderLimit){
      motor.set(0);
    }
    else {
      motor.set(TurretConfig.counterClockwiseSpeed);
    }
  }

  public void stop() {
    motor.set(0);
  }
}
