// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DioPort;
import frc.robot.config.DriveConfig;
import frc.robot.config.PwmPort;

public class DriveSubsystem extends SubsystemBase {

  PWMSparkMax leftFrontMotor = new PWMSparkMax(PwmPort.LeftFrontDriveMotor);
  PWMSparkMax leftRearMotor = new PWMSparkMax(PwmPort.LeftRearDriveMotor);

  PWMSparkMax rightFrontMotor = new PWMSparkMax(PwmPort.RightFrontDriveMotor);
  PWMSparkMax rightRearMotor = new PWMSparkMax(PwmPort.RightRearDriveMotor);

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup (leftFrontMotor, leftRearMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

  DifferentialDrive driveTrain = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  private Encoder leftEncoder = new Encoder(DioPort.LeftEncoderChannelA, DioPort.LeftEncoderChannelB);
  private Encoder rightEncoder = new Encoder(DioPort.RightEncoderChannelA, DioPort.RightEncoderChannelB);

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()));
  private Field2d field = new Field2d();

  private DriveSimulation simulation;

  public DriveSubsystem() {
    rightControllerGroup.setInverted(true);
    driveTrain.setMaxOutput(DriveConfig.maxOutput);

    initEncoders();
    initDashboard();

    if (RobotBase.isSimulation()) {
      simulation = new DriveSimulation(leftEncoder, rightEncoder, gyro);
    }
  }

  @Override
  public void periodic() {

    odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );

    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
  }

  public void simulationPeriodic() {
    simulation.update(leftControllerGroup.get(), rightControllerGroup.get());
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllerGroup.setVoltage(leftVolts);
    rightControllerGroup.setVoltage(rightVolts);
    driveTrain.feed();
  }

  private void initEncoders() {
    double metersPerPulse = 2 * DriveConfig.wheelRadiusInMeters * Math.PI / DriveConfig.encoderResolution;

    leftEncoder.setDistancePerPulse(metersPerPulse);
    leftEncoder.setReverseDirection(true);
    rightEncoder.setDistancePerPulse(metersPerPulse);
  }

  private void initDashboard() {
    SmartDashboard.putData("Field", field);
  }
}