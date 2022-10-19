// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.CoolDownTimer;

public class DriveBackwardsCommand extends CommandBase {

  DrivetrainSubsystem drivetrainSubsystem = RobotContainer.drivetrainSubsystem;
  CoolDownTimer coolDownTimer;
  double seconds;

  public DriveBackwardsCommand(double seconds) {
    addRequirements(drivetrainSubsystem);

    this.seconds = seconds;
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.zeroGyroscope();
    coolDownTimer = new CoolDownTimer((long) seconds * 1000);
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
          -0.25 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          0,
          0,
          drivetrainSubsystem.getGyroscopeRotation()
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
          0,
          0,
          0,
          drivetrainSubsystem.getGyroscopeRotation()
      )
    );
  }

  @Override
  public boolean isFinished() {
    return coolDownTimer.isCool();
  }
}
