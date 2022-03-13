// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SorterSubsystem;

public class ManualIntake extends CommandBase {

  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
  SorterSubsystem sorterSubsystem = RobotContainer.sorterSubsystem;

  public ManualIntake() {
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.intake();
    sorterSubsystem.in();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    sorterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
