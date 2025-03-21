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
import java.util.function.Supplier;

public class AlignAndScore {

  /**
   * Command that aligns the robot to the pole and scores if the preset is correct.
   *
   * @param drive The drive subsystem.
   * @param coralSystem The coral handling system.
   * @param isLeftPole Whether the target pole is on the left side.
   * @return The full command sequence.
   */
  public static Command create(Drive drive, CoralSystem coralSystem, boolean isLeftPole) {

    // Supplier to dynamically compute the target pose
    Supplier<Pose2d> targetPoseSupplier =
        () -> {
          AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
          if (selection == null || selection.getAcceptedFaceId() == null) {
            System.out.println("No valid reef face found. Cancelling alignment.");
            return null;
          }

          double closestDistance = selection.getAcceptedDistance();
          if (closestDistance > FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE) {
            System.out.println("Reef face too far: " + closestDistance);
            return null;
          }

          // Check if we are scoring at L1
          boolean isScoringL1 = coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE;

          Pose2d targetPose;
          if (isScoringL1) {
            targetPose = calculateL1Pose(drive, selection.getAcceptedFaceId());
          } else {
            targetPose = calculatePolePose(drive, selection.getAcceptedFaceId(), isLeftPole);
          }

          if (targetPose == null) {
            System.out.println("Failed to calculate valid target pose. Cancelling.");
            return null;
          }

          System.out.println("Driving to target pose: " + targetPose);
          return targetPose;
        };

    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting drive to pole...")),

        // Compute the target pose and either drive or skip alignment
        Commands.either(
                Commands.runOnce(
                    () ->
                        System.out.println(
                            "Skipping alignment, no valid target.")), // Skip if no valid target
                new DriveToPoseNoJoystick(drive, targetPoseSupplier), // Drive if pose is valid
                () -> targetPoseSupplier.get() == null // Condition: if null, skip
                )

            // Stop execution if alignment wasn't successful
            .unless(() -> targetPoseSupplier.get() == null),

        // Log when alignment is done
        Commands.runOnce(() -> System.out.println("Robot aligned, checking presets...")),

        // Auto-score only if preset matches, otherwise do nothing
        Commands.either(
            new ScoreCoralInTeleopCommand(coralSystem.getIntake()), // Auto-score command
            Commands.none(), // No scoring if preset is not correct
            () ->
                inScoringConfiguration(coralSystem)
                    && drive.getReefFaceSelection().getTagSeenRecently()
                    && ((BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
                                == BranchAlignmentStatus.GREEN
                            && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.L1_SCORE)
                        || (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_SCORE))));
  }

  /** Calculates the target pose for pole alignment using correct alliance logic. */
  private static Pose2d calculatePolePose(Drive drive, int faceId, boolean isLeftPole) {
    return DriveToCommands.calculatePolePose(drive, faceId, isLeftPole);
  }

  private static Pose2d calculateL1Pose(Drive drive, int faceId) {
    return DriveToCommands.calculateL1Pose(drive, faceId);
  }

  public static boolean inScoringConfiguration(CoralSystem coralSystem) {
    return coralSystem.getCurrentCoralPreset() == coralSystem.getTargetCoralPreset()
        && (coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L2
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L3
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L4);
  }
}
