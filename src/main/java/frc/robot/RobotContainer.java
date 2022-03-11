// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.auto.BackOffLineAuto;
import frc.robot.commands.auto.FourBallStraightAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.climber.ExtendArms;
import frc.robot.commands.climber.RetractArms;
import frc.robot.commands.climber.StopArms;
import frc.robot.commands.climber.ToggleArmPositions;
import frc.robot.commands.climber.EnableClimber;
import frc.robot.commands.drive.ShiftHighGear;
import frc.robot.commands.drive.ShiftLowGear;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.Outtake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.ShootFar;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.commands.shooterfeed.ShooterFeedDown;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUpV2;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.commands.turret.EnableAutoAim;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.commands.turret.DriveBy;
import frc.robot.config.roborio.JoystickPort;
import frc.robot.config.subsystems.TurretConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        driveSubsystem.setDefaultCommand(
            new RunCommand(() -> driveSubsystem.tankDrive(-leftPilotJoystick.getY(), -rightPilotJoystick.getY(), leftPilotJoystick.getRawAxis(2)), driveSubsystem)
        );

        configurePilotButtonBindings();
        configureCopilotButtonBindings();

        initCamera();

        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        autoChooser.addOption("Back Off Line Path", new BackOffLineAuto(driveSubsystem, sorterSubsystem, shooterSubsystem, turretSubsystem, shooterFeedSubsystem));
        //autoChooser.addOption("Back Off Line DR", new BackOffLineDeadReckoning(driveSubsystem));
        autoChooser.addOption("Two Ball", new TwoBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem, turretSubsystem));
        autoChooser.addOption("Four Ball Straight", new FourBallStraightAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem, turretSubsystem));
        //autoChooser.addOption("Four Ball", new FourBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem));
        //autoChooser.addOption("Test", new TestAuto(driveSubsystem));


        SmartDashboard.putData(autoChooser);
    }


    private void configurePilotButtonBindings() {

        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 1);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 2);
        JoystickButton retractIntakeButton = new JoystickButton(rightPilotJoystick, 1);

        JoystickButton shiftLowGearButton = new JoystickButton(leftPilotJoystick, 4);
        JoystickButton shiftHighGearButton = new JoystickButton(leftPilotJoystick, 5);

        JoystickButton enableClimberButton1 = new JoystickButton(rightPilotJoystick, 8);
        JoystickButton enableClimberButton2 = new JoystickButton(rightPilotJoystick, 9);

        intakeButton
            .whileHeld(new ParallelCommandGroup(
                new Intake(intakeSubsystem),
                new SorterIn(sorterSubsystem)
            ))
            .whenReleased(new ParallelCommandGroup(
                new StopIntake(intakeSubsystem),
                new SorterStop(sorterSubsystem)
            ));

        outtakeButton
            .whileHeld(new Outtake(intakeSubsystem))
            .whenReleased(new StopIntake(intakeSubsystem));

        retractIntakeButton
            .whenPressed(new RetractIntake(intakeSubsystem));

        shiftLowGearButton
            .whenPressed(new ShiftLowGear(driveSubsystem));

        shiftHighGearButton
            .whenPressed(new ShiftHighGear(driveSubsystem));

        enableClimberButton1.and(enableClimberButton2)
            .whenActive(new EnableClimber(climberSubsystem, turretSubsystem));

        // autoClimbButton
        //     .whenPressed(new AutoClimb(climberSubsystem));
    }

    private void configureCopilotButtonBindings() {

        JoystickButton toggleShooterButton = new JoystickButton(copilotGamepad, 3);

        JoystickButton shootMediumButton = new JoystickButton(copilotGamepad, 1);
        JoystickButton shootFarButton = new JoystickButton(copilotGamepad, 4);

        JoystickButton driveByCenterButton = new JoystickButton(copilotGamepad, 2);
        JoystickButton driveByLeftButton = new JoystickButton(copilotGamepad, 9);
        JoystickButton driveByRightButton = new JoystickButton(copilotGamepad, 10);

        JoystickButton tiltArmsButton = new JoystickButton(copilotGamepad, 12);

        JoystickButton toggleClimberLocksButton = new JoystickButton(copilotGamepad, 11);

        Trigger extendArmsTrigger = new Trigger(() -> copilotGamepad.getLeftY() < -0.5);
        Trigger retractArmsTrigger = new Trigger(() -> copilotGamepad.getLeftY() > 0.5);

        JoystickButton shooterFeedUpButton = new JoystickButton(copilotGamepad, 6);
        JoystickButton shooterFeedDownButton = new JoystickButton(copilotGamepad, 8);

        JoystickButton bumpShooterRpmUpButton = new JoystickButton(copilotGamepad, 5);
        JoystickButton bumpShooterRpmDownButton = new JoystickButton(copilotGamepad, 7);

        Trigger homeTurretTrigger = new Trigger(() -> copilotGamepad.getPOV() == 0);
        Trigger rotateTurretClockwiseTrigger = new Trigger(() -> copilotGamepad.getPOV() == 90);
        Trigger enableAutoAimTrigger = new Trigger(() -> copilotGamepad.getPOV() == 180);
        Trigger rotateTurretCounterClockwiseTrigger = new Trigger(() -> copilotGamepad.getPOV() == 270);


        toggleShooterButton
            .whenPressed(new ToggleShooter(shooterSubsystem));

        shootMediumButton
            .whenPressed(new ShootMedium(shooterSubsystem, turretSubsystem));

        shootFarButton
            .whenPressed(new ShootFar(shooterSubsystem, turretSubsystem));

        tiltArmsButton
            .whenPressed(new ToggleArmPositions(climberSubsystem));

        extendArmsTrigger
            .whenActive(new ExtendArms(climberSubsystem))
            .whenInactive(new StopArms(climberSubsystem));

        retractArmsTrigger
            .whenActive(new RetractArms(climberSubsystem))
            .whenInactive(new StopArms(climberSubsystem));

        toggleClimberLocksButton
            .whenPressed(new InstantCommand(climberSubsystem::toggleArmLocks));

        shooterFeedUpButton
            .whenHeld(new ParallelCommandGroup(
                new SorterIn(sorterSubsystem),
                new ShooterFeedUpV2(shooterFeedSubsystem, shooterSubsystem)
            ))
            .whenReleased(new ParallelCommandGroup(
                new SorterStop(sorterSubsystem),
                new ShooterFeedStop(shooterFeedSubsystem)
            ));

        shooterFeedDownButton
            .whenPressed(new ShooterFeedDown(shooterFeedSubsystem))
            .whenReleased(new ShooterFeedStop(shooterFeedSubsystem));

        bumpShooterRpmUpButton
            .whenPressed(new InstantCommand(shooterSubsystem::bumpRpmUp));

        bumpShooterRpmDownButton
            .whenPressed(new InstantCommand(shooterSubsystem::bumpRpmDown));

        homeTurretTrigger
            .whenActive(new MoveTurretToDegrees(180, turretSubsystem));

        rotateTurretClockwiseTrigger
            .whenActive(new InstantCommand(turretSubsystem::rotateClockwise, turretSubsystem))
            .whenInactive(new InstantCommand(turretSubsystem::stop, turretSubsystem));

        enableAutoAimTrigger
            .whenActive(new EnableAutoAim(turretSubsystem));

        rotateTurretCounterClockwiseTrigger
            .whenActive(new InstantCommand(turretSubsystem::rotateCounterClockwise, turretSubsystem))
            .whenInactive(new InstantCommand(turretSubsystem::stop, turretSubsystem));

        driveByCenterButton
            .whenPressed(new DriveBy(TurretConfig.degreesAtCenter, turretSubsystem, shooterSubsystem));

        driveByLeftButton
            .whenPressed(new DriveBy(TurretConfig.degreesAtLeft, turretSubsystem, shooterSubsystem));

        driveByRightButton
            .whenPressed(new DriveBy(TurretConfig.degreesAtRight, turretSubsystem, shooterSubsystem));
    }

    private void initCamera(){

        if (RobotBase.isSimulation()) return;

        UsbCamera backCam = CameraServer.startAutomaticCapture();
        backCam.setResolution(160, 120);
        backCam.setFPS(20);
        backCam.setExposureAuto();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
