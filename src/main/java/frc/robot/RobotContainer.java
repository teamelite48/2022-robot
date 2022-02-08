// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.config.JoystickPort;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    private final Command extendLeftArm = new InstantCommand(() -> climberSubsystem.extendLeftArm(), climberSubsystem);
    private final Command retractLeftArm = new InstantCommand(() -> climberSubsystem.retractLeftArm(), climberSubsystem);
    private final Command stopLeftArm = new InstantCommand(() -> climberSubsystem.stopLeftArm(), climberSubsystem);

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(tankDrive);

        configurePilotButtonBindings();
        configureCopilotButtonBindings();
    }

    private void configurePilotButtonBindings() {

        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 6);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 11);
        JoystickButton retractIntakeButton = new JoystickButton(rightPilotJoystick, 10);

        intakeButton
            .whenPressed(intake)
            .whenReleased(stopIntake);

        outtakeButton
            .whileHeld(outtake)
            .whenInactive(stopIntake);

        retractIntakeButton
            .whenPressed(retractIntake);
    }

    private void configureCopilotButtonBindings() {

        JoystickButton shootButton = new JoystickButton(copilotGamepad, 2);

        JoystickButton enableClimberButtonA = new JoystickButton(copilotGamepad, 7);
        JoystickButton enableClimberButtonB = new JoystickButton(copilotGamepad, 8);

        JoystickButton toggleLeftArmPositionButton = new JoystickButton(copilotGamepad, 9);
        JoystickButton toggleRightArmPositionButton = new JoystickButton(copilotGamepad, 10);

        Trigger extendLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() < -0.5);
        Trigger retractLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() > 0.5);

        // TODO: Add triggers to extend and retract the right arm.

        shootButton
            .whileHeld(shoot);

        enableClimberButtonA.and(enableClimberButtonB)
            .whenActive(enableClimber);

        toggleLeftArmPositionButton
            .whenPressed(toggleLeftArmPosition);

        toggleRightArmPositionButton
            .whenPressed(toggleRightArmPosition);

        extendLeftArmTrigger
            .whenActive(extendLeftArm)
            .whenInactive(stopLeftArm);

        retractLeftArmTrigger
            .whenActive(retractLeftArm)
            .whenInactive(stopLeftArm);

        // TODO: Bind the extend and retract commands to the extend and retract buttons for the right arm.
        //       Don't forget to stop the arm when the triggers are inactive.
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
