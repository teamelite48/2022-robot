// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  PS4Controller pilotInput = new PS4Controller(0);
  
  ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();

  ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  Command intake = new RunCommand(() -> intakeSubsystem.intake(), intakeSubsystem); 
  Command outtake = new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem);
  Command stopIntake = new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem);
  Command drive = new RunCommand(() -> driveSubsystem.tankDrive(pilotInput.getLeftY(), pilotInput.getRightY()), driveSubsystem);

  //creates field for simmulation
  private Field2d field = new Field2d();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(drive);

    configureButtonBindings();

    SmartDashboard.putData("Field", field);
  }

  private void configureButtonBindings() {
    JoystickButton circleButton = new JoystickButton(pilotInput, 1);
    
    circleButton.whenPressed(intake);
    circleButton.whenReleased(stopIntake);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
