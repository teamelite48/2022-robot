package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.subsystems.DriveSubsystem;

public class BackOffLineAuto extends SequentialCommandGroup {

    public BackOffLineAuto(
        DriveSubsystem driveSubsystem
    ) {
        RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);

        addCommands(
            new ResetOdometry(11, 4, 180, driveSubsystem),
            ramseteCommandFactory.createCommand(TrajectoryType.BackOffLine)
        );
    }
}
