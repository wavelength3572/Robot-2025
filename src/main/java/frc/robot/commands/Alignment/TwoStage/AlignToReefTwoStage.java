package frc.robot.commands.Alignment.TwoStage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToCommands;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RobotStatus;
import org.littletonrobotics.junction.Logger;

public class AlignToReefTwoStage extends SequentialCommandGroup {
  private final Timer runtimeTimer = new Timer();

  public AlignToReefTwoStage(Drive drive, int reefFaceId, boolean isLeftPole) {
    Pose2d reefPose = getTargetPose(drive, reefFaceId, isLeftPole);
    if (reefPose == null) {
      Logger.recordOutput(
          "AlignAndScoreTwoStage/Error", "Target pose was null, skipping sequence.");
      return;
    }

    Pose2d standoffPose = DriveToCommands.calculateStandoffPose(reefPose, 0.25);

    Logger.recordOutput("AlignAndScoreTwoStage/ReefFaceId", reefFaceId);
    Logger.recordOutput("AlignAndScoreTwoStage/ReefPose", reefPose);
    Logger.recordOutput("AlignAndScoreTwoStage/StandoffPose", standoffPose);

    addCommands(
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  runtimeTimer.reset();
                  runtimeTimer.start();
                }),
            new DriveToPosePP(drive, standoffPose).withTimeout(2),
            new FinalAlign(drive, reefPose).withTimeout(0.5),
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "AlignAndScoreTwoStage/TotalDurationSeconds", runtimeTimer.get()))));
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
