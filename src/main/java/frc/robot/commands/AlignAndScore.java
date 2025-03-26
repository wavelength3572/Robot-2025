package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.BranchAlignmentUtils;
import frc.robot.util.BranchAlignmentUtils.BranchAlignmentStatus;
import org.littletonrobotics.junction.Logger;

/** Command that aligns the robot to the pole and then scores if the preset is correct. */
public class AlignAndScore {

  /**
   * Create the full command sequence that attempts to align the robot to a pole (or L1 scoring
   * position) and then score if the coral system is in a scoring preset.
   */
  public static Command create(Drive drive, CoralSystem coralSystem, boolean isLeftPole) {
    // Compute the target pose ONCE at the start
    Pose2d targetPose = computeTargetPoseOrNull(drive, coralSystem, isLeftPole);

    return Commands.sequence(
        // 1) Mark that we initialized
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScore/Stage", "INIT")),

        // 2) If targetPose == null, skip alignment. Otherwise, run DriveToPose.
        Commands.either(
                // --- Skip alignment ---
                Commands.runOnce(
                    () -> {
                      Logger.recordOutput("AlignAndScore/Stage", "SKIP_ALIGNMENT");
                    }),

                // --- Drive to the target pose ---
                new DriveToPose(drive, () -> targetPose)
                    .beforeStarting(
                        () -> {
                          Logger.recordOutput("AlignAndScore/Stage", "DRIVE_TO_POSE");
                          Logger.recordOutput("AlignAndScore/DesiredPose", targetPose);
                        }),

                // Condition: skip if the pose is null
                () -> targetPose == null)
            // Also skip the Drive command if targetPose was null
            .unless(() -> targetPose == null),

        // 3) Mark that we're done driving
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScore/Stage", "POST_DRIVE")),

        // 4) Wait until the coral system is at its goal
        new TimedWaitUntilCommand("inPresetConfiguration", coralSystem::isAtGoal),

        // 5) Auto-score if conditions match, otherwise skip
        Commands.either(
            // --- Scoring branch ---
            Commands.sequence(
                Commands.runOnce(() -> Logger.recordOutput("AlignAndScore/Stage", "SCORING")),
                new ScoreCoralInTeleopCommand(coralSystem.getIntake()),
                Commands.runOnce(() -> Logger.recordOutput("AlignAndScore/Stage", "SCORING_DONE"))),
            // --- No scoring branch ---
            Commands.runOnce(() -> Logger.recordOutput("AlignAndScore/Stage", "NO_SCORING")),
            // Condition: can we score?
            () ->
                inScoringConfiguration(coralSystem)
                    && drive.getReefFaceSelection().getTagSeenRecently()
                    && ((BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
                                == BranchAlignmentStatus.GREEN
                            && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.L1_SCORE)
                        || (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_SCORE))));
  }

  /**
   * Determines the target Pose2d to drive to. Returns null if no valid face or if the face is too
   * far. Only called once.
   */
  private static Pose2d computeTargetPoseOrNull(
      Drive drive, CoralSystem coralSystem, boolean isLeftPole) {

    Logger.recordOutput("AlignAndScore/Stage", "COMPUTE_POSE");

    AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
    if (selection == null || selection.getAcceptedFaceId() == null) {
      Logger.recordOutput("AlignAndScore/ComputePose/Status", "NO_FACE");
      return null;
    }

    double closestDistance = selection.getAcceptedDistance();
    Logger.recordOutput("AlignAndScore/ComputePose/FaceID", selection.getAcceptedFaceId());
    Logger.recordOutput("AlignAndScore/ComputePose/Distance", closestDistance);

    // Check distance threshold
    if (closestDistance > FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE) {
      Logger.recordOutput("AlignAndScore/ComputePose/Status", "FACE_TOO_FAR");
      return null;
    }

    // Check if we are scoring at L1
    boolean isScoringL1 = (coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE);
    Logger.recordOutput("AlignAndScore/ComputePose/IsScoringL1", isScoringL1);

    // Compute the desired pose (pole vs. L1)
    Pose2d targetPose =
        isScoringL1
            ? calculateL1Pose(drive, selection.getAcceptedFaceId())
            : calculatePolePose(drive, selection.getAcceptedFaceId(), isLeftPole);

    if (targetPose == null) {
      Logger.recordOutput("AlignAndScore/ComputePose/Status", "CALC_FAILED");
      return null;
    }

    // Log the pose
    Logger.recordOutput("AlignAndScore/ComputePose/Status", "POSE_VALID");
    Logger.recordOutput("AlignAndScore/ComputePose/TargetPose", targetPose);

    return targetPose;
  }

  /** Calculates the target pose for pole alignment using correct alliance logic. */
  private static Pose2d calculatePolePose(Drive drive, int faceId, boolean isLeftPole) {
    return DriveToCommands.calculatePolePose(drive, faceId, isLeftPole);
  }

  private static Pose2d calculateL1Pose(Drive drive, int faceId) {
    return DriveToCommands.calculateL1Pose(drive, faceId);
  }

  /**
   * Returns true if the coral system is in a "scoring" configuration (e.g. L1_SCORE, L2, L3, L4).
   */
  public static boolean inScoringConfiguration(CoralSystem coralSystem) {
    return coralSystem.getCurrentCoralPreset() == coralSystem.getTargetCoralPreset()
        && (coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L2
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L3
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L4);
  }
}
