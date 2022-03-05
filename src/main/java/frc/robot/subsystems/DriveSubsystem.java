// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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
import frc.robot.config.roborio.DioPort;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.roborio.PwmPort;
import frc.robot.config.subsystems.DriveConfig;
import frc.robot.config.sysid.SysIdConfig;
import frc.robot.simulation.DriveSimulation;

public class DriveSubsystem extends SubsystemBase {

  PWMSparkMax leftFrontMotor = new PWMSparkMax(PwmPort.LeftFrontDriveMotor);
  PWMSparkMax leftMidMotor = new PWMSparkMax(PwmPort.LeftMidDriveMotor);
  PWMSparkMax leftRearMotor = new PWMSparkMax(PwmPort.LeftRearDriveMotor);

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup (leftFrontMotor, leftMidMotor, leftRearMotor);

  PWMSparkMax rightFrontMotor = new PWMSparkMax(PwmPort.RightFrontDriveMotor);
  PWMSparkMax rightMidMotor = new PWMSparkMax(PwmPort.RightMidDriveMotor);
  PWMSparkMax rightRearMotor = new PWMSparkMax(PwmPort.RightRearDriveMotor);

  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor,rightMidMotor, rightRearMotor);

  DifferentialDrive driveTrain = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  Solenoid shifterSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.GearShift);

  private Encoder leftEncoder = new Encoder(DioPort.LeftEncoderChannelA, DioPort.LeftEncoderChannelB);
  private Encoder rightEncoder = new Encoder(DioPort.RightEncoderChannelA, DioPort.RightEncoderChannelB);

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()));
  private Field2d field = new Field2d();

  double throttleValue = DriveConfig.maxOutput;
  Long lastShiftMillis = null;

  DriveSimulation simulation;

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

    if (lastShiftMillis != null) {
      Long currentMillis = System.currentTimeMillis();

      if (currentMillis - lastShiftMillis > DriveConfig.shiftCoolDownMillis) {
        lastShiftMillis = null;
      }
    }
    else {
      driveTrain.setMaxOutput(Math.min(throttleValue, DriveConfig.maxOutput));
    }

    odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );

    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    SmartDashboard.putString("Gear", shifterSolenoid.get() == DriveConfig.lowGearValue ? "Low" : "High");
    SmartDashboard.putNumber("Left Speed", leftControllerGroup.get());
    SmartDashboard.putNumber("Right Speed", rightControllerGroup.get());
    SmartDashboard.putNumber("Throttle Value", throttleValue);
  }

  public void simulationPeriodic() {
    simulation.update(leftControllerGroup.get(), rightControllerGroup.get());
  }

  public void tankDrive(double leftSpeed, double rightSpeed, double throttleValue) {
    this.throttleValue = Math.max(DriveConfig.maxOutput * ((throttleValue * -1) + 1) / 2, DriveConfig.minOutput);

    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }
  public void shiftHighGear() {
    driveTrain.setMaxOutput(DriveConfig.shiftingMaxOutput);

    shifterSolenoid.set(DriveConfig.highGearValue);

    lastShiftMillis = System.currentTimeMillis();
  }

  public void shiftLowGear() {
    driveTrain.setMaxOutput(DriveConfig.shiftingMaxOutput);

    shifterSolenoid.set(DriveConfig.lowGearValue);

    lastShiftMillis = System.currentTimeMillis();
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

  public void resetOdometry(double x, double y, double degrees) {
    leftEncoder.reset();
    rightEncoder.reset();

    odometry.resetPosition(new Pose2d(x, y, new Rotation2d()), Rotation2d.fromDegrees(degrees));

    if (RobotBase.isSimulation()) {
      simulation = new DriveSimulation(leftEncoder, rightEncoder, gyro);
    }
  }

  private void initEncoders() {
    double metersPerPulse = 2 * SysIdConfig.wheelRadiusInMeters * Math.PI / DriveConfig.encoderResolution;

    leftEncoder.setDistancePerPulse(metersPerPulse);
    rightEncoder.setDistancePerPulse(metersPerPulse);

    leftEncoder.setReverseDirection(true);
  }

  private void initDashboard() {
    SmartDashboard.putData("Field", field);
  }
}
