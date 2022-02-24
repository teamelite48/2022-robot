// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.TurretConfig;
import frc.robot.config.roborio.CanBusId;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CanBusId.TurretMotor, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  public TurretSubsystem() {
    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Postiton", encoder.getPosition());
  }

  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public void rotateClockwise() {
    motor.setVoltage(TurretConfig.clockwiseSpeed);
  }

  public void rotateCounterClockwise() {
    motor.setVoltage(TurretConfig.counterClockwiseSpeed);
  }

  public void stop() {
    motor.setVoltage(0);
  }

  // method to auto aim
  // if the target is in view than try to lock on
  // calcluate new speed
  // adjust motor if the desired speed won't turn the turret outside of it's allowed range
}
