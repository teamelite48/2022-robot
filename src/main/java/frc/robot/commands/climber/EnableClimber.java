// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.DisableTurret;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.config.subsystems.TurretConfig;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class EnableClimber extends SequentialCommandGroup {

  ClimberSubsystem climberSubsystem = RobotContainer.climberSubsystem;
  TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;

  public EnableClimber() {
    addCommands(
      new MoveTurretToDegrees(TurretConfig.degreesAtCenter),
      new DisableTurret(),
      new InstantCommand(climberSubsystem::enableClimber)
    );
  }
}
