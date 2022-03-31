package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.pathfollowing.PathType;

public class TwoBallShortAuto extends SequentialCommandGroup {

    public TwoBallShortAuto() {
        addCommands(
            new ResetOdometry(8.8, 6.5, 100),
            new AutoIntake(),
            new SorterIn(),
            new InstantCommand(RobotContainer.shooterSubsystem::setMediumSpeed),
            new InstantCommand(RobotContainer.shooterSubsystem::shooterOn),
            new WaitCommand(1),
            new FollowPath(PathType.TwoBallShort1),
            new InstantCommand(RobotContainer.turretSubsystem::turnAutoAimOn), 
            new WaitCommand(0.5),
            new ShooterFeedUp().withTimeout(3),
            new ShooterOff()
        );
    }
}