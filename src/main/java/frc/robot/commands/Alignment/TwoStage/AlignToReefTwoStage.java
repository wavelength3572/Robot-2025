package frc.robot.commands.Alignment.TwoStage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private final double STANDOFF_DISTANCE = 0.20; // Meters to stand off from the reef face.

  private final Timer runtimeTimer = new Timer();
  private boolean drivingBackwards = false;

  public AlignToReefTwoStage(Drive drive, int reefFaceId, boolean isLeftPole) {
    Pose2d reefPose = getTargetPose(drive, reefFaceId, isLeftPole);
    if (reefPose == null) {
      Logger.recordOutput(
          "AlignAndScoreTwoStage/Error", "Target pose was null, skipping sequence.");
      return;
    }

    Pose2d standoffPose = DriveToCommands.calculateStandoffPose(reefPose, STANDOFF_DISTANCE);
    Translation2d acceptedFace = RobotStatus.getReefFaceSelection().getAcceptedFace();
    Translation2d currentTranslation = RobotStatus.getRobotPose().getTranslation();
    boolean flipStartTangent =
        isRobotBetween(acceptedFace, standoffPose.getTranslation(), currentTranslation);

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
            new DriveToPosePP(drive, standoffPose, drivingBackwards, flipStartTangent)
                .withTimeout(3),
            new FinalAlign(drive, reefPose).withTimeout(0.7),
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "AlignAndScoreTwoStage/TotalDurationSeconds", runtimeTimer.get()))));
  }

  /**
   * Returns true if the robot's current translation lies between pointA and pointB along the line.
   */
  private static boolean isRobotBetween(
      Translation2d pointA, Translation2d pointB, Translation2d robotPos) {
    Translation2d segment = pointB.minus(pointA);
    Translation2d toRobot = robotPos.minus(pointA);
    double segmentLengthSquared = dot(segment, segment);
    if (segmentLengthSquared == 0) {
      return false;
    }
    double projection = dot(toRobot, segment) / segmentLengthSquared;
    return projection > 0 && projection < 1;
  }

  // Helper to compute dot product
  private static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  private Pose2d getTargetPose(Drive drive, int reefFaceId, boolean isLeftBranch) {

    if (RobotStatus.getTargetPreset() == CoralSystemPresets.L1_SCORE) {
      drivingBackwards = true;
      return DriveToCommands.calculateL1Pose(drive, reefFaceId, isLeftBranch);
    } else {
      drivingBackwards = false;
      return DriveToCommands.calculatePolePose(drive, reefFaceId, isLeftBranch);
    }
  }

  public static Command alignToReefTwoStage(Drive drive, int reefFaceId, boolean isLeftPole) {
    return new AlignToReefTwoStage(drive, reefFaceId, isLeftPole);
  }
}
