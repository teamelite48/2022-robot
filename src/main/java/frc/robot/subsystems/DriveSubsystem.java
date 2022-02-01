// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.PneumaticChannel;
import frc.robot.config.DioPort;
import frc.robot.config.DriveConfig;
import frc.robot.config.PwmPort;

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

  Solenoid leftShifterSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftGearShiftForward);
  Solenoid rightShifterSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightGearShiftForward);

  private Encoder leftEncoder = new Encoder(DioPort.LeftEncoderChannelA, DioPort.LeftEncoderChannelB);
  private Encoder rightEncoder = new Encoder(DioPort.RightEncoderChannelA, DioPort.RightEncoderChannelB);

  private AnalogGyro gyro = new AnalogGyro(1);

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

  private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

  private DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    LinearSystemId.identifyDrivetrainSystem(DriveConfig.kvLinear, DriveConfig.kaLinear, DriveConfig.kvAngular, DriveConfig.kaAngular, DriveConfig.TrackWidthInMeters),
    DCMotor.getNEO(3),
    DriveConfig.GearingReduction,
    DriveConfig.TrackWidthInMeters,
    DriveConfig.WheelRadiusInMeters,
    DriveConfig.EnocoderNoise
  );

  private Field2d field = new Field2d();

  public DriveSubsystem() {
    rightControllerGroup.setInverted(true);
    driveTrain.setMaxOutput(DriveConfig.MaxOutput);

    initEncoders();
    initDashboard();
  }

  @Override
  public void periodic() {

    odometry.update(
      gyro.getRotation2d(),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );

    field.setRobotPose(odometry.getPoseMeters());
  }

  public void simulationPeriodic() {

    driveSim.setInputs(
      leftControllerGroup.get() * RobotController.getInputVoltage(),
      rightControllerGroup.get() * RobotController.getInputVoltage()
    );

    driveSim.update(0.02);

    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());

    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());

    gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }
  public void shiftHighGear() {
    leftShifterSolenoid.set(true);
    rightShifterSolenoid.set(true);
  }

  public void shiftLowGear() {
    leftShifterSolenoid.set(false);
    rightShifterSolenoid.set(false);
  }

  private void initEncoders() {
    double metersPerPulse = 2 * DriveConfig.WheelRadiusInMeters * Math.PI / DriveConfig.EncoderResolution;

    leftEncoder.setDistancePerPulse(metersPerPulse);
    rightEncoder.setDistancePerPulse(metersPerPulse);
  }

  private void initDashboard() {
    SmartDashboard.putData("Field", field);
  }
}
