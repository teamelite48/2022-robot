package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.pathfollowing.PathType;

public class BackOffLineAuto extends SequentialCommandGroup {

    public BackOffLineAuto() {
        addCommands(
            new ResetOdometry(10.5, 3.45, 180),
            new InstantCommand(RobotContainer.shooterSubsystem::setMediumSpeed),
            new ShooterOn(),
            new InstantCommand(RobotContainer.shooterSubsystem::turnRangeBasedRPMOn),
            new FollowPath(PathType.BackOffLine),
            new InstantCommand(RobotContainer.turretSubsystem::turnAutoAimOn),
            new SorterIn(),
            new ShooterFeedUp().withTimeout(2),
            new ShooterOff()
        );
    }
}
