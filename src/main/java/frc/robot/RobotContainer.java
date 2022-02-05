// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.config.JoystickPort;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final Joystick leftPilotJoystick = new Joystick(JoystickPort.LeftPilotJoystick);
    private final Joystick rightPilotJoystick = new Joystick(JoystickPort.RightPilotJoystick);
    private final XboxController copilotGamepad = new XboxController(JoystickPort.CopilotGamepad);

    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private final Command tankDrive = new RunCommand(() -> driveSubsystem.tankDrive(-leftPilotJoystick.getY(), -rightPilotJoystick.getY()), driveSubsystem);
    private final Command intake = new RunCommand(() -> intakeSubsystem.intake(), intakeSubsystem);
    private final Command outtake = new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem);
    private final Command stopIntake = new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem);
    private final Command retractIntake = new InstantCommand(() -> intakeSubsystem.retract(), intakeSubsystem);
    private final Command shoot = new RunCommand(() -> shooterSubsystem.shoot(), shooterSubsystem);
   
    private final Command enableClimber = new InstantCommand ( ()-> climberSubsystem.enableClimber(), climberSubsystem);
    private final Command toggleLeftArmPosition = new InstantCommand(() -> climberSubsystem.toggleLeftArmPosition(), climberSubsystem);
    private final Command toggleRightArmPosition = new InstantCommand(() -> climberSubsystem.toggleRightArmPosition(), climberSubsystem);
    private final Command adjustArmLengths = new RunCommand(() -> climberSubsystem.adjustArmLengths(-copilotGamepad.getLeftY(), -copilotGamepad.getRightY()), climberSubsystem);

    // creates field for simmulation
    private Field2d field = new Field2d();



    public RobotContainer() {
        driveSubsystem.setDefaultCommand(tankDrive);
        climberSubsystem.setDefaultCommand(adjustArmLengths);    

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 6);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 11);
        JoystickButton retractIntakeButton = new JoystickButton(rightPilotJoystick, 10);
        JoystickButton toggleLeftArmPositionButton = new JoystickButton(copilotGamepad, 9);
        JoystickButton toggleRightArmPositionButton = new JoystickButton(copilotGamepad, 10);
        JoystickButton shootButton = new JoystickButton(copilotGamepad, 2);
        JoystickButton enableClimberButtonA = new JoystickButton(copilotGamepad, 7);
        JoystickButton enableClimberButtonB = new JoystickButton(copilotGamepad, 8);
    
        intakeButton
            .whenPressed(intake)
            .whenReleased(stopIntake);

        outtakeButton
            .whileHeld(outtake)
            .whenInactive(stopIntake);

        retractIntakeButton
            .whenPressed(retractIntake);

        shootButton
            .whileHeld(shoot);
        
        toggleLeftArmPositionButton
            .whenPressed(toggleLeftArmPosition);

        toggleRightArmPositionButton
            .whenPressed(toggleRightArmPosition);

        enableClimberButtonA.and(enableClimberButtonB)
            .whenActive(enableClimber);
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
