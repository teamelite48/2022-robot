package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.pathfollowing.PathType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {


    public TwoBallAuto(
        DriveSubsystem driveSubsystem,
        IntakeSubsystem intakeSubsystem,
        SorterSubsystem sorterSubsystem,
        ShooterSubsystem shooterSubsystem,
        ShooterFeedSubsystem shooterFeedSubsystem,
        TurretSubsystem turretSubsystem
    ) {
        addCommands(
            new ResetOdometry(10.3, 2.75, 200, driveSubsystem),
            new InstantCommand(intakeSubsystem::deploy, intakeSubsystem),
            new WaitCommand(.5),
            new InstantCommand(intakeSubsystem::intake, intakeSubsystem),
            new SorterIn(sorterSubsystem),
            new ShootMedium(shooterSubsystem, turretSubsystem),
            new ShooterOn(shooterSubsystem),
            new FollowPath(PathType.TwoBall1),
            new ShooterFeedUp(shooterFeedSubsystem),
            new WaitCommand(2),
            new RetractIntake(intakeSubsystem),
            new WaitCommand(2),
            new ShooterFeedStop(shooterFeedSubsystem),
            new SorterStop(sorterSubsystem),
            new ShooterOff(shooterSubsystem)
        );
    }
}