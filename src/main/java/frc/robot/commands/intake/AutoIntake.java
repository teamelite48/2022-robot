// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.config.subsystems.IntakeConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.CoolDownTimer;


public class AutoIntake extends CommandBase {

  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
  CoolDownTimer coolDown = new CoolDownTimer(IntakeConfig.deployIntakeCooldown + 100);

  public AutoIntake() {
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return coolDown.isCool();
  }
}
