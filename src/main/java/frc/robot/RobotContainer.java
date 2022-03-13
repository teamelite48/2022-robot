// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Joysticks.LogitechGamepad;
import frc.robot.Joysticks.LogitechJoystick;
import frc.robot.commands.auto.BackOffLineAuto;
import frc.robot.commands.auto.FourBallStraightAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.auto.TwoBallShortAuto;
import frc.robot.commands.climber.ExtendArms;
import frc.robot.commands.climber.RetractArms;
import frc.robot.commands.climber.StopArms;
import frc.robot.commands.climber.ToggleArmPositions;
import frc.robot.commands.climber.EnableClimber;
import frc.robot.commands.drive.ShiftHighGear;
import frc.robot.commands.drive.ShiftLowGear;
import frc.robot.commands.intake.IntakeV2;
import frc.robot.commands.intake.OuttakeV2;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootFar;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.commands.shooterfeed.ShooterFeedDown;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUpV2;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final ShooterFeedSubsystem shooterFeedSubsystem = new ShooterFeedSubsystem();
    private final SorterSubsystem sorterSubsystem = new SorterSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();

    private final LogitechJoystick leftJoystick = new LogitechJoystick(JoystickPort.LeftPilotJoystick);
    private final LogitechJoystick rightJoystick = new LogitechJoystick(JoystickPort.RightPilotJoystick);
    private final LogitechGamepad gamepad = new LogitechGamepad(JoystickPort.CopilotGamepad);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        driveSubsystem.setDefaultCommand(
            new RunCommand(() -> driveSubsystem.tankDrive(-leftJoystick.getY(), -rightJoystick.getY(), leftJoystick.getRawAxis(2)), driveSubsystem)
        );

        configurePilotButtonBindings();
        configureCopilotButtonBindings();

        initializeCamera();
        inititialzeAutoChooser();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configurePilotButtonBindings() {

        leftJoystick.getTrigger()
            .whenHeld(new IntakeV2(intakeSubsystem, sorterSubsystem));

        rightJoystick.getButton2()
            .whenHeld(new OuttakeV2(intakeSubsystem));

        rightJoystick.getTrigger()
            .whenPressed(new RetractIntake(intakeSubsystem));

        leftJoystick.getButton4()
            .whenPressed(new ShiftLowGear(driveSubsystem));

        leftJoystick.getButton5()
            .whenPressed(new ShiftHighGear(driveSubsystem));

        rightJoystick.getButton8().and(rightJoystick.getButton9())
            .whenActive(new EnableClimber(climberSubsystem, turretSubsystem));
    }

    private void configureCopilotButtonBindings() {

        gamepad.getBButton()
            .whenPressed(new ToggleShooter(shooterSubsystem));

        gamepad.getXButton()
            .whenPressed(new ShootMedium(shooterSubsystem, turretSubsystem));

        gamepad.getYButton()
            .whenPressed(new ShootFar(shooterSubsystem, turretSubsystem));

        gamepad.getRightStickButton()
            .whenPressed(new ToggleArmPositions(climberSubsystem));

        gamepad.getLeftStickButton()
            .whenPressed(new InstantCommand(climberSubsystem::toggleArmLocks));

        gamepad.getRightBumper()
            .whenHeld(new ShooterFeedUpV2(shooterFeedSubsystem, shooterSubsystem, sorterSubsystem));

        gamepad.getRightTrigger()
            .whenPressed(new ShooterFeedDown(shooterFeedSubsystem))
            .whenReleased(new ShooterFeedStop(shooterFeedSubsystem));

        gamepad.getLeftBumper()
            .whenPressed(new InstantCommand(shooterSubsystem::bumpRpmUp));

        gamepad.getLeftTrigger()
            .whenPressed(new InstantCommand(shooterSubsystem::bumpRpmDown));

        gamepad.getDpadUpTrigger()
            .whenActive(new MoveTurretToDegrees(180, turretSubsystem));

        gamepad.getDpadRightTrigger()
            .whenActive(new InstantCommand(turretSubsystem::rotateClockwise, turretSubsystem))
            .whenInactive(new InstantCommand(turretSubsystem::stop, turretSubsystem));

        gamepad.getDpadDownTrigger()
            .whenActive(new EnableAutoAim(turretSubsystem));

        gamepad.getDpadLeftTrigger()
            .whenActive(new InstantCommand(turretSubsystem::rotateCounterClockwise, turretSubsystem))
            .whenInactive(new InstantCommand(turretSubsystem::stop, turretSubsystem));

        gamepad.getAButton()
            .whenPressed(new DriveBy(TurretConfig.degreesAtCenter, turretSubsystem, shooterSubsystem));

        gamepad.getBackButton()
            .whenPressed(new DriveBy(TurretConfig.degreesAtLeft, turretSubsystem, shooterSubsystem));

        gamepad.getStartButton()
            .whenPressed(new DriveBy(TurretConfig.degreesAtRight, turretSubsystem, shooterSubsystem));

        new Trigger(() -> gamepad.getLeftY() < -0.5)
            .whenActive(new ExtendArms(climberSubsystem))
            .whenInactive(new StopArms(climberSubsystem));

        new Trigger(() -> gamepad.getLeftY() > 0.5)
            .whenActive(new RetractArms(climberSubsystem))
            .whenInactive(new StopArms(climberSubsystem));
    }

    private void initializeCamera(){

        if (RobotBase.isSimulation()) return;

        UsbCamera backCam = CameraServer.startAutomaticCapture();
        backCam.setResolution(160, 120);
        backCam.setFPS(20);
        backCam.setExposureAuto();
    }

    private void inititialzeAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        autoChooser.addOption("Back Off Line Path", new BackOffLineAuto(driveSubsystem, sorterSubsystem, shooterSubsystem, turretSubsystem, shooterFeedSubsystem));
        //autoChooser.addOption("Back Off Line DR", new BackOffLineDeadReckoning(driveSubsystem));
        autoChooser.addOption("Two Ball", new TwoBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem, turretSubsystem));
        autoChooser.addOption("Two Ball Short", new TwoBallShortAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem, turretSubsystem));
        autoChooser.addOption("Four Ball Straight", new FourBallStraightAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem, turretSubsystem));
        //autoChooser.addOption("Four Ball", new FourBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem));
        //autoChooser.addOption("Test", new TestAuto(driveSubsystem));


        SmartDashboard.putData(autoChooser);
    }
}
