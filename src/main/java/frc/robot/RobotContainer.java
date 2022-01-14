// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class RobotContainer {
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Command intakeCommand = new RunCommand(() -> intakeSubsystem.intake(), intakeSubsystem); 
  private final Command outtakeCommand = new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem);
  private final Command stopOuttakeCommand = new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem);


  public RobotContainer() {
    intakeSubsystem.setDefaultCommand(intakeCommand);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
