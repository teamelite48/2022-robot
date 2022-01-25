// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.PneumaticChannel;
import frc.robot.config.PwmPort;

public class DriveSubsystem extends SubsystemBase {

  // TODO: 2 additional left motors
  // TODO: 2 additional right motors
  // TODO: add new motors to controller groups
  // TODO: add right solenoid

  PWMSparkMax leftFrontMotor = new PWMSparkMax(PwmPort.LeftFrontDriveMotor);
  MotorControllerGroup leftControllerGroup = new MotorControllerGroup (leftFrontMotor);

  PWMSparkMax rightFrontMotor = new PWMSparkMax(PwmPort.RightFrontDriveMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor);

  DifferentialDrive driveTrain = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  Solenoid leftShifterSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftGearShiftForward);
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leftControllerGroup.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }
  public void shiftHighGear() {
    leftShifterSolenoid.set(true);
    // TODO: also activate right solenoid 
  }

  // TODO: method to shift into low gear
}
