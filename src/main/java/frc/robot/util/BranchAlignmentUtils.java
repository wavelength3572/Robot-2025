package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public final class BranchAlignmentUtils {

  public enum BranchAlignmentStatus {
    NONE,
    RED,
    GREEN
  }

  // ----------------- TUNE THESE DISTANCES (meters) -----------------
  // Threshold for forward alignment—if the robot isn’t within this distance,
  // we consider it not aligned regardless of lateral offset.
  private static final double FORWARD_THRESHOLD = 0.10;

  // Lateral thresholds for traffic-light style alignment.
  public static final double LATERAL_THRESHOLD_RED = 0.150;
  public static final double LATERAL_THRESHOLD_SOLID_GREEN =
      0.025; // this is only used to make the blinking solid

  // ----------------- OFFSET VALUES (meters) -----------------
  // Offsets for scoring positions relative to the AprilTag pose.
  // One for the left pole and one for the right pole.

  //x is distance from the face to center of robot, y is the lateral distance from the face.
  private static final Transform2d RIGHT_POLE_OFFSET =
      new Transform2d(new Translation2d(Units.inchesToMeters(17.51), Units.inchesToMeters(-0.787)), new Rotation2d(0));
  private static final Transform2d LEFT_POLE_OFFSET =
      new Transform2d(new Translation2d(Units.inchesToMeters(17.51), Units.inchesToMeters(-13.582)), new Rotation2d(0));

  @Getter
  public static BranchAlignmentStatus currentBranchAlignmentStatus = BranchAlignmentStatus.NONE;

  @Getter public static double lateralErrorToNearestPole = Double.POSITIVE_INFINITY;

  private BranchAlignmentUtils() {
    // prevent instantiation
  }

  /**
   * Returns how close the robot is to the nearest branch (pole) on a given reef face. This method
   * computes the scoring poses for both left and right poles by applying offsets to the AprilTag
   * pose (using plus() to combine poses), logs both sets of relative pose data, and then selects
   * the scoring pose that is closest to the robot for determining the alignment status.
   *
   * <p>The robot's pose is transformed into the scoring coordinate frame where: - x:
   * forward/backward offset (normal to the face) - y: lateral (left/right) offset.
   *
   * @param robotPose The robot's global Pose2d.
   * @param faceId The reef face’s AprilTag ID.
   * @return BranchAlignmentStatus indicating the alignment quality.
   */

  // TODO: consider making the green blink frequency change depending on how close you are?
  public static BranchAlignmentStatus getBranchAlignmentStatus(Pose2d robotPose, int faceId) {
    // Retrieve the reef face pose directly from the AprilTag layout.
    Optional<Pose3d> reefFacePose3d = VisionConstants.aprilTagLayout.getTagPose(faceId);

    // If we don't have a reef face or if we aren't in a scoring preset, then don't do this work.
    if (!reefFacePose3d.isPresent()
        || (!RobotStatus.haveCoral())
        || (RobotStatus.getCurrentPreset() != CoralSystemPresets.L1
            && RobotStatus.getCurrentPreset() != CoralSystemPresets.L2
            && RobotStatus.getCurrentPreset() != CoralSystemPresets.L3
            && RobotStatus.getCurrentPreset() != CoralSystemPresets.L4)) {
      Logger.recordOutput("Alignment/Branch/Status", BranchAlignmentStatus.NONE.toString());
      currentBranchAlignmentStatus = BranchAlignmentStatus.NONE;
      return currentBranchAlignmentStatus;
    }

    // Convert the AprilTag pose to 2d.
    Pose2d tagPose2d = reefFacePose3d.get().toPose2d();

    // Use plus() to apply the scoring offsets.
    Pose2d leftScoringPose = tagPose2d.plus(LEFT_POLE_OFFSET);
    Pose2d rightScoringPose = tagPose2d.plus(RIGHT_POLE_OFFSET);

    // Transform the robot's pose relative to both scoring poses.
    Pose2d leftRelativePose = robotPose.relativeTo(leftScoringPose);
    Pose2d rightRelativePose = robotPose.relativeTo(rightScoringPose);

    double leftForwardError = leftRelativePose.getTranslation().getX();
    double leftLateralError = leftRelativePose.getTranslation().getY();
    double rightForwardError = rightRelativePose.getTranslation().getX();
    double rightLateralError = rightRelativePose.getTranslation().getY();

    // Log both sets of scoring data.
    Logger.recordOutput("Alignment/Branch/LeftScoringPose", leftScoringPose);
    Logger.recordOutput("Alignment/Branch/RightScoringPose", rightScoringPose);
    Logger.recordOutput("Alignment/Branch/LeftForwardError", leftForwardError);
    Logger.recordOutput("Alignment/Branch/LeftLateralError", leftLateralError);
    Logger.recordOutput("Alignment/Branch/RightForwardError", rightForwardError);
    Logger.recordOutput("Alignment/Branch/RightLateralError", rightLateralError);

    // Determine which scoring pose is closer to the robot.
    double leftDistance = robotPose.getTranslation().getDistance(leftScoringPose.getTranslation());
    double rightDistance =
        robotPose.getTranslation().getDistance(rightScoringPose.getTranslation());

    Pose2d chosenRelativePose;
    String chosenSide;
    if (leftDistance < rightDistance) {
      chosenRelativePose = leftRelativePose;
      lateralErrorToNearestPole = Math.abs(leftLateralError);
      chosenSide = "Left";
    } else {
      chosenRelativePose = rightRelativePose;
      lateralErrorToNearestPole = Math.abs(rightLateralError);
      chosenSide = "Right";
    }

    Logger.recordOutput("Alignment/Branch/ChosenSide", chosenSide);
    Logger.recordOutput("Alignment/Branch/LateralErrorToChosenPole", lateralErrorToNearestPole);

    // Use the chosen relative pose to decide alignment.
    double forwardOffset = chosenRelativePose.getTranslation().getX();
    double lateralOffset = chosenRelativePose.getTranslation().getY();

    // Check the forward alignment.
    if (Math.abs(forwardOffset) > FORWARD_THRESHOLD) {
      Logger.recordOutput("Alignment/Branch/Status", BranchAlignmentStatus.NONE.toString());
      currentBranchAlignmentStatus = BranchAlignmentStatus.NONE;
      return currentBranchAlignmentStatus;
    }

    // Evaluate the lateral offset to decide the traffic-light status.
    if (Math.abs(lateralOffset) > LATERAL_THRESHOLD_RED) {
      Logger.recordOutput("Alignment/Branch/Status", BranchAlignmentStatus.RED.toString());
      currentBranchAlignmentStatus = BranchAlignmentStatus.RED;
    } else {
      Logger.recordOutput("Alignment/Branch/Status", BranchAlignmentStatus.GREEN.toString());
      currentBranchAlignmentStatus = BranchAlignmentStatus.GREEN;
    }
    return currentBranchAlignmentStatus;
  }
}
