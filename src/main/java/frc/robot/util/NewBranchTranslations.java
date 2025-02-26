package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public final class NewBranchTranslations {

  // Adjust these offset values based on your field measurements.
  // They represent the transformation from the AprilTag pose to the branch (pole) scoring position.
  private static final Transform2d RIGHT_POLE_OFFSET =
      new Transform2d(new Translation2d(.445, .001), new Rotation2d(0));
  private static final Transform2d LEFT_POLE_OFFSET =
      new Transform2d(new Translation2d(.445, -.345), new Rotation2d(0));

  private NewBranchTranslations() {
    // Prevent instantiation.
  }

  /**
   * Computes the left branch translation based on the AprilTag pose for the given faceId.
   *
   * @param faceId The AprilTag ID for the reef face.
   * @return The Translation2d of the left branch (pole), or null if the AprilTag pose isn’t
   *     available.
   */
  public static Translation2d getLeftBranchTranslation(int faceId) {
    Optional<Pose3d> tagPoseOpt = VisionConstants.aprilTagLayout.getTagPose(faceId);
    if (!tagPoseOpt.isPresent()) {
      Logger.recordOutput(
          "NewBranchTranslations/Left", "AprilTag pose not available for faceId: " + faceId);
      return null;
    }
    // Convert to 2D and apply the left offset using plus().
    Pose2d tagPose2d = tagPoseOpt.get().toPose2d();
    Pose2d leftScoringPose = tagPose2d.plus(LEFT_POLE_OFFSET);
    Logger.recordOutput("NewBranchTranslations/LeftScoringPose", leftScoringPose);
    return leftScoringPose.getTranslation();
  }

  /**
   * Computes the right branch translation based on the AprilTag pose for the given faceId.
   *
   * @param faceId The AprilTag ID for the reef face.
   * @return The Translation2d of the right branch (pole), or null if the AprilTag pose isn’t
   *     available.
   */
  public static Translation2d getRightBranchTranslation(int faceId) {
    Optional<Pose3d> tagPoseOpt = VisionConstants.aprilTagLayout.getTagPose(faceId);
    if (!tagPoseOpt.isPresent()) {
      Logger.recordOutput(
          "NewBranchTranslations/Right", "AprilTag pose not available for faceId: " + faceId);
      return null;
    }
    // Convert to 2D and apply the right offset using plus().
    Pose2d tagPose2d = tagPoseOpt.get().toPose2d();
    Pose2d rightScoringPose = tagPose2d.plus(RIGHT_POLE_OFFSET);
    Logger.recordOutput("NewBranchTranslations/RightScoringPose", rightScoringPose);
    return rightScoringPose.getTranslation();
  }
}
