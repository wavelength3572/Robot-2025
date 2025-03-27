package frc.robot.commands.Alignment.TwoStage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToCommands;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RobotStatus;

public class AlignToReefTwoStage extends SequentialCommandGroup {

  public AlignToReefTwoStage(Drive drive, int reefFaceId, boolean isLeftPole) {
    Pose2d reefPose = getTargetPose(drive, reefFaceId, isLeftPole);
    if (reefPose == null) return;

    Pose2d standoffPose = DriveToCommands.calculateStandoffPose(reefPose, 0.25); // ~10 inches back

    addCommands(
        new DriveToPosePP(drive, standoffPose).withTimeout(2),
        new FinalAlign(drive, reefPose).withTimeout(.5));
  }

  private Pose2d getTargetPose(Drive drive, int reefFaceId, boolean isLeftBranch) {
    return RobotStatus.getTargetPreset() == CoralSystemPresets.L1_SCORE
        ? DriveToCommands.calculateL1Pose(drive, reefFaceId)
        : DriveToCommands.calculatePolePose(drive, reefFaceId, isLeftBranch);
  }

  public static Command alignToReefTwoStage(Drive drive, int reefFaceId, boolean isLeftPole) {
    return new AlignToReefTwoStage(drive, reefFaceId, isLeftPole);
  }
}
