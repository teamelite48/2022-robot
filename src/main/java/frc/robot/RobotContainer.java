// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Joysticks.LogitechGamepad;
import frc.robot.Joysticks.LogitechJoystick;
import frc.robot.Joysticks.PS4Gamepad;
import frc.robot.commands.auto.BackOffLineAuto;
import frc.robot.commands.auto.FourBallStraightAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.auto.TwoBallShortAuto;
import frc.robot.commands.climber.ExtendArms;
import frc.robot.commands.climber.RetractArms;
import frc.robot.commands.climber.ToggleArmPositions;
import frc.robot.commands.climber.EnableClimber;
import frc.robot.commands.drive.ShiftHighGear;
import frc.robot.commands.drive.ShiftLowGear;
import frc.robot.commands.intake.ManualIntake;
import frc.robot.commands.intake.Outtake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootNear;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShootFar;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooterfeed.ShooterFeedDown;
import frc.robot.commands.shooterfeed.ManualShooterFeedUp;
import frc.robot.commands.turret.EnableAutoAim;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.config.roborio.JoystickPort;
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

    public static DriveSubsystem driveSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static ClimberSubsystem climberSubsystem;
    public static ShooterFeedSubsystem shooterFeedSubsystem;
    public static SorterSubsystem sorterSubsystem;
    public static TurretSubsystem turretSubsystem;

    final LogitechJoystick leftJoystick = new LogitechJoystick(JoystickPort.LeftPilotJoystick);
    final LogitechJoystick rightJoystick = new LogitechJoystick(JoystickPort.RightPilotJoystick);
    final LogitechGamepad gamepad = new LogitechGamepad(JoystickPort.CopilotGamepad);

    final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        driveSubsystem = new DriveSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        shooterFeedSubsystem = new ShooterFeedSubsystem();
        sorterSubsystem = new SorterSubsystem();
        turretSubsystem = new TurretSubsystem();

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

        leftJoystick.getTrigger().whenHeld(new ManualIntake());
        leftJoystick.getButton4().whenPressed(new ShiftLowGear());
        leftJoystick.getButton5().whenPressed(new ShiftHighGear());

        rightJoystick.getTrigger().whenPressed(new RetractIntake());
        rightJoystick.getButton2().whenHeld(new Outtake());
        rightJoystick.getButton8().and(rightJoystick.getButton9()).whenActive(new EnableClimber());
    }

    private void configureCopilotButtonBindings() {

        gamepad.getLeftBumper().whenPressed(new InstantCommand(shooterSubsystem::bumpRpmUp));
        gamepad.getLeftTrigger().whenPressed(new InstantCommand(shooterSubsystem::bumpRpmDown));

        gamepad.getRightBumper().whenHeld(new ManualShooterFeedUp());
        gamepad.getRightTrigger().whenHeld(new ShooterFeedDown());

        gamepad.getDpadUpTrigger().whenActive(new MoveTurretToDegrees(180));
        gamepad.getDpadDownTrigger().whenActive(new EnableAutoAim());

        gamepad.getDpadLeftTrigger()
            .whenActive(new InstantCommand(turretSubsystem::rotateCounterClockwise, turretSubsystem))
            .whenInactive(new InstantCommand(turretSubsystem::stop, turretSubsystem));

        gamepad.getDpadRightTrigger()
            .whenActive(new InstantCommand(turretSubsystem::rotateClockwise, turretSubsystem))
            .whenInactive(new InstantCommand(turretSubsystem::stop, turretSubsystem));

        gamepad.getLeftStickButton().whenPressed(new InstantCommand(climberSubsystem::toggleArmLocks));
        gamepad.getRightStickButton().whenPressed(new ToggleArmPositions());

        new Trigger(() -> gamepad.getLeftY() < -0.5).whileActiveOnce(new ExtendArms());
        new Trigger(() -> gamepad.getLeftY() > 0.5).whileActiveOnce(new RetractArms());

        gamepad.getAButton().whenPressed(new ShootNear());
        gamepad.getBButton().whenPressed(new ShooterOff());
        gamepad.getXButton().whenPressed(new ShootMedium());
        gamepad.getYButton().whenPressed(new ShootFar());

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
        autoChooser.addOption("Back Off Line Path", new BackOffLineAuto());
        autoChooser.addOption("Two Ball", new TwoBallAuto());
        autoChooser.addOption("Two Ball Short", new TwoBallShortAuto());
        autoChooser.addOption("Four Ball Straight", new FourBallStraightAuto());
        //autoChooser.addOption("Four Ball", new FourBallAuto());
        //autoChooser.addOption("Test", new TestAuto());

        SmartDashboard.putData(autoChooser);
    }
}
