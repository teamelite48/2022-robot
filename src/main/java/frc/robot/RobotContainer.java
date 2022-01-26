// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.config.JoystickPort;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final Joystick leftPilotJoystick = new Joystick(JoystickPort.LeftPilotJoystick);
    private final Joystick rightPilotJoystick = new Joystick(JoystickPort.RightPilotJoystick);

    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private final Command intake = new RunCommand(() -> intakeSubsystem.intake(), intakeSubsystem);
    private final Command outtake = new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem);
    private final Command stopIntake = new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem);
    private final Command drive = new RunCommand(() -> driveSubsystem.tankDrive(leftPilotJoystick.getY(), rightPilotJoystick.getY()), driveSubsystem);

    // creates field for simmulation
    private Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(drive);

        configureButtonBindings();

        SmartDashboard.putData("Field", field);
    }

    private void configureButtonBindings() {
        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 6);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 11);
        
        intakeButton
            .whenPressed(intake)
            .whenReleased(stopIntake);

        outtakeButton
            .whenPressed(outtake)
            .whenReleased(stopIntake);
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
