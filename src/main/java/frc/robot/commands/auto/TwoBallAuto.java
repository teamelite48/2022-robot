package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.Intake;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {


    public TwoBallAuto(
        DriveSubsystem driveSubsystem,
        IntakeSubsystem intakeSubsystem,
        SorterSubsystem sorterSubsystem,
        ShooterSubsystem shooterSubsystem,
        ShooterFeedSubsystem shooterFeedSubsystem
    ) {

        RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);

        addCommands(
            new ResetOdometry(10.5, 3.1, -136, driveSubsystem),
            new Intake(intakeSubsystem)
        );
    }
}