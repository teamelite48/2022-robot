// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.config.roborio.JoystickPort;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    private final Joystick leftPilotJoystick = new Joystick(JoystickPort.LeftPilotJoystick);
    private final Joystick rightPilotJoystick = new Joystick(JoystickPort.RightPilotJoystick);
    private final XboxController copilotGamepad = new XboxController(JoystickPort.CopilotGamepad);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final ShooterFeedSubsystem shooterFeedSubsystem = new ShooterFeedSubsystem();
    private final SorterSubsystem sorterSubsystem = new SorterSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();

    private final RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);

    private final Command tankDrive = new RunCommand(() -> driveSubsystem.tankDrive(-leftPilotJoystick.getY(), -rightPilotJoystick.getY()), driveSubsystem);
    private final Command shiftLowGear = new InstantCommand(() -> driveSubsystem.shiftLowGear());
    private final Command shiftHighGear = new InstantCommand(() -> driveSubsystem.shiftHighGear());

    private final Command intake = new InstantCommand(() -> intakeSubsystem.intake(), intakeSubsystem);
    private final Command outtake = new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem);
    private final Command stopIntake = new InstantCommand(() -> intakeSubsystem.stop(), intakeSubsystem);
    private final Command retractIntake = new InstantCommand(() -> intakeSubsystem.retract(), intakeSubsystem);

    private final Command toggleShooter = new InstantCommand(() -> shooterSubsystem.toggleShooter(), shooterSubsystem);
    private final Command shootNear = new InstantCommand(() -> shooterSubsystem.setLowSpeed(), shooterSubsystem);
    private final Command shootMedium = new InstantCommand(() -> shooterSubsystem.setMediumSpeed(), shooterSubsystem);
    private final Command shootFar = new InstantCommand(() -> shooterSubsystem.setHighSpeed(), shooterSubsystem);

    private final Command toggleClimberEnabled = new InstantCommand (()-> climberSubsystem.toggleClimberEnabled(), climberSubsystem);
    private final Command toggleLeftArmPosition = new InstantCommand(() -> climberSubsystem.toggleLeftArmPosition(), climberSubsystem);
    private final Command toggleRightArmPosition = new InstantCommand(() -> climberSubsystem.toggleRightArmPosition(), climberSubsystem);
    private final Command extendLeftArm = new InstantCommand(() -> climberSubsystem.extendLeftArm(), climberSubsystem);
    private final Command retractLeftArm = new InstantCommand(() -> climberSubsystem.retractLeftArm(), climberSubsystem);
    private final Command stopLeftArm = new InstantCommand(() -> climberSubsystem.stopLeftArm(), climberSubsystem);
    private final Command extendRightArm = new InstantCommand(() -> climberSubsystem.extendRightArm(), climberSubsystem);
    private final Command retractRightArm = new InstantCommand(() -> climberSubsystem.retractRightArm(), climberSubsystem);
    private final Command stopRightArm = new InstantCommand(() -> climberSubsystem.stopRightArm(), climberSubsystem);

    private final Command shooterFeedUp = new InstantCommand(() -> shooterFeedSubsystem.up(), shooterFeedSubsystem);
    private final Command shooterFeedDown = new InstantCommand(() -> shooterFeedSubsystem.down(), shooterFeedSubsystem);
    private final Command shooterFeedStop = new InstantCommand(() -> shooterFeedSubsystem.stop(), shooterFeedSubsystem);

    private final Command sorterIn = new InstantCommand(() -> sorterSubsystem.in(), sorterSubsystem);
    private final Command sorterOut = new InstantCommand(() -> sorterSubsystem.out(), sorterSubsystem);
    private final Command sorterStop = new InstantCommand(() -> sorterSubsystem.stop(), sorterSubsystem);

    private final Command rotateTurretClockwise = new InstantCommand(() -> turretSubsystem.rotateClockwise(), turretSubsystem);
    private final Command rotateTurretCounterClockwise = new InstantCommand(() -> turretSubsystem.rotateCounterClockwise(), turretSubsystem);
    private final Command stopTurret = new InstantCommand(() -> turretSubsystem.stop(), turretSubsystem);

    private final Command pickUpCargoAndShoot = new SequentialCommandGroup(
        ramseteCommandFactory.createCommand(TrajectoryType.GetReadyToShoot),
        new InstantCommand(() -> shooterFeedSubsystem.up(), shooterFeedSubsystem)
    );

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(tankDrive);

        configurePilotButtonBindings();
        configureCopilotButtonBindings();
    }

    private void configurePilotButtonBindings() {

        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 6);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 11);
        JoystickButton retractIntakeButton = new JoystickButton(rightPilotJoystick, 10);

        JoystickButton shiftLowGearButton = new JoystickButton(leftPilotJoystick, 4);
        JoystickButton shiftHighGearButton = new JoystickButton(leftPilotJoystick, 5);
        JoystickButton shootButton = new JoystickButton(copilotGamepad, 2);

        intakeButton
            .whenPressed(intake)
            .whenReleased(stopIntake);

        outtakeButton
            .whileHeld(outtake)
            .whenInactive(stopIntake);

        retractIntakeButton
            .whenPressed(retractIntake);

        shiftLowGearButton
            .whenPressed(shiftLowGear);

        shiftHighGearButton
            .whenPressed(shiftHighGear);

    }

    private void configureCopilotButtonBindings() {

        JoystickButton toggleShooterButton = new JoystickButton(copilotGamepad, 2);

        JoystickButton shootNearButton = new JoystickButton(copilotGamepad, 1);
        JoystickButton shootMediumButton = new JoystickButton(copilotGamepad, 3);
        JoystickButton shootFarButton = new JoystickButton(copilotGamepad, 4);

        JoystickButton enableClimberButton1 = new JoystickButton(copilotGamepad, 7);
        JoystickButton enableClimberButton2 = new JoystickButton(copilotGamepad, 8);

        JoystickButton toggleLeftArmPositionButton = new JoystickButton(copilotGamepad, 9);
        JoystickButton toggleRightArmPositionButton = new JoystickButton(copilotGamepad, 10);

        Trigger extendLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() < -0.5);
        Trigger retractLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() > 0.5);

        Trigger extendRightArmTrigger = new Trigger(() -> copilotGamepad.getRightY() < -0.5);
        Trigger retractRightArmTrigger = new Trigger(() -> copilotGamepad.getRightY() > 0.5);

        JoystickButton shooterFeedUpButton = new JoystickButton(copilotGamepad, 6);
        Trigger shooterFeedDownTrigger = new Trigger(() -> copilotGamepad.getRawAxis(3) > 0.5);

        JoystickButton sorterInButton = new JoystickButton(copilotGamepad, 5);
        Trigger sorterOutTrigger = new Trigger(() -> copilotGamepad.getRawAxis(2) > 0.5);

        JoystickButton rotateTurretClockwiseButton = new JoystickButton(copilotGamepad, 6);
        JoystickButton rotateTurretCounterClockwiseButton = new JoystickButton(copilotGamepad, 5);

        toggleShooterButton
            .whenPressed(toggleShooter);

        shootNearButton
            .whenPressed(shootNear);

        shootMediumButton
            .whenPressed(shootMedium);

        shootFarButton
            .whenPressed(shootFar);

        enableClimberButton1.and(enableClimberButton2)
            .whenActive(toggleClimberEnabled);

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

        extendRightArmTrigger
            .whenActive(extendRightArm)
            .whenInactive(stopRightArm);

        retractRightArmTrigger
            .whenActive(retractRightArm)
            .whenInactive(stopRightArm);

        shooterFeedUpButton
            .whenPressed(shooterFeedUp)
            .whenReleased(shooterFeedStop);

        shooterFeedDownTrigger
            .whenActive(shooterFeedDown)
            .whenInactive(shooterFeedStop);

        sorterInButton
            .whenPressed(sorterIn)
            .whenReleased(sorterStop);

        sorterOutTrigger
            .whenActive(sorterOut)
            .whenInactive(sorterStop);

        rotateTurretClockwiseButton
            .whenPressed(rotateTurretClockwise)
            .whenReleased(stopTurret);

        rotateTurretCounterClockwiseButton
            .whenPressed(rotateTurretCounterClockwise)
            .whenReleased(stopTurret);
    }

    public Command getAutonomousCommand() {
        return pickUpCargoAndShoot;
    }
}
