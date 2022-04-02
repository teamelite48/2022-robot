// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.subsystems.ShooterConfig;

public class ShooterSubsystem extends SubsystemBase {

  final WPI_TalonFX rearMotor = new WPI_TalonFX(CanBusId.RearShooterMotor);
  final WPI_TalonFX frontMotor = new WPI_TalonFX(CanBusId.FrontShooterMotor);

  final PIDController frontPIDController = new PIDController(ShooterConfig.frontkP, ShooterConfig.frontkI, ShooterConfig.frontkD);
  final SimpleMotorFeedforward frontFeedForward = new SimpleMotorFeedforward(ShooterConfig.frontks, ShooterConfig.frontkv, ShooterConfig.frontka);

  final PIDController rearPIDController = new PIDController(ShooterConfig.rearkP, ShooterConfig.rearkI, ShooterConfig.rearkD);
  final SimpleMotorFeedforward rearFeedForward = new SimpleMotorFeedforward(ShooterConfig.rearks, ShooterConfig.rearkv, ShooterConfig.rearka);

  boolean isShooterOn = false;
  double targetRPM = ShooterConfig.mediumRPM;
  boolean isRangeBasedRPMOn = false;

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  final NetworkTableEntry ty = table.getEntry("ty");

  double distanceToTargetInFeet = 0.0;

  public ShooterSubsystem() {

    rearMotor.configFactoryDefault();
    frontMotor.configFactoryDefault();

    SmartDashboard.putNumber("Current Rear RPM", 0);
    SmartDashboard.putNumber("Current Front RPM", 0);
  }

  @Override
  public void periodic() {

    double angleToGoalInDegrees = ShooterConfig.limelightAngleInDegrees + (ty.getDouble(0.0));
    double angleToGoalInRadians = Math.toRadians(angleToGoalInDegrees);

    distanceToTargetInFeet = Math.min(ShooterConfig.limelightToVisionTargetInFeet / Math.tan(angleToGoalInRadians), ShooterConfig.maxDistanceInFeet);

    if (isShooterOn == false) {
      frontMotor.set(0);
      rearMotor.set(0);
    }
    else {

      if(isRangeBasedRPMOn){
        targetRPM = getRPMByDistanceToTarget();
      }

      setFrontMotor(targetRPM);
      setRearMotor(targetRPM * 2.5);
    }

    SmartDashboard.putNumber("Target RPM", targetRPM);
    SmartDashboard.putBoolean("Shooter On", isShooterOn);
    SmartDashboard.putNumber("Distance to Target", distanceToTargetInFeet);
  }

  private void setRearMotor(double targetRPM) {
    double rearCurrentRPM = -1 * (rearMotor.getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048;

    rearMotor.setVoltage(rearFeedForward.calculate(targetRPM / 60) + rearPIDController.calculate(rearCurrentRPM, targetRPM));

    SmartDashboard.putNumber("Current Rear RPM", rearCurrentRPM * -1);
  }

  private void setFrontMotor(double targetRPM) {
    double frontCurrentRPM = -1 * (frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048;

    frontMotor.setVoltage(frontFeedForward.calculate(targetRPM / 60) + frontPIDController.calculate(frontCurrentRPM, targetRPM));

    SmartDashboard.putNumber("Current Front RPM", frontCurrentRPM * -1);
  }

  public void toggleShooter() {
    isShooterOn = !isShooterOn;
  }

  public void shooterOn() {
    isShooterOn = true;
  }

  public void shooterOff() {
    isRangeBasedRPMOn = false;
    isShooterOn = false;
  }

  public void setLowSpeed() {
    isRangeBasedRPMOn = false;
    targetRPM = ShooterConfig.lowRPM;
  }

  public void setMediumSpeed() {
    isRangeBasedRPMOn = false;
    targetRPM = ShooterConfig.mediumRPM;
  }

  public double getRPMByDistanceToTarget() {

    double bottomRPM = ShooterConfig.distanceToRPMMap.getOrDefault(Math.floor(distanceToTargetInFeet), 0.0);
    double topRPM = ShooterConfig.distanceToRPMMap.getOrDefault(Math.ceil(distanceToTargetInFeet), 0.0);

    targetRPM = bottomRPM + ((distanceToTargetInFeet - Math.floor(distanceToTargetInFeet)) * (topRPM - bottomRPM));

    return targetRPM;
  }

  public void turnRangeBasedRPMOn() {
    isRangeBasedRPMOn = true;
  }

  public void turnRangeBasedRPMOff() {
    isRangeBasedRPMOn = false;
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
