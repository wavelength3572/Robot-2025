package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

public class AlignAndScore {

  public static Command create(Drive drive, CoralSystem coralSystem, boolean isLeftPole,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier) {
    Supplier<Pose2d> targetPoseSupplier = createTargetPoseSupplier(drive, coralSystem, isLeftPole);

    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting AlignAndScore sequence...")),

        Commands.sequence(
            new HybridHeadingAlignCommand(
                drive,
                () -> {
                  Pose2d pose = targetPoseSupplier.get();
                  return pose != null ? pose.getRotation() : drive.getRotation();
                },
                xJoystickSupplier,
                yJoystickSupplier),
            new DriveToPoseNoJoystick(drive, targetPoseSupplier)).unless(() -> targetPoseSupplier.get() == null),

        Commands.runOnce(() -> System.out.println("Robot aligned, checking presets...")),

        new TimedWaitUntilCommand("inPresetConfiguration", coralSystem::isAtGoal),

        Commands.either(
            new ScoreCoralInTeleopCommand(coralSystem.getIntake()),
            Commands.none(),
            () -> AlignAndScore.inScoringConfiguration(coralSystem)
                && drive.getReefFaceSelection().getTagSeenRecently()
                && ((BranchAlignmentUtils.getCurrentBranchAlignmentStatus() == BranchAlignmentStatus.GREEN
                    && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.L1_SCORE)
                    || (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_SCORE))));
  }

  private static Supplier<Pose2d> createTargetPoseSupplier(
      Drive drive, CoralSystem coralSystem, boolean isLeftPole) {
    return () -> {
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

      boolean isScoringL1 = coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE;
      Pose2d pose = isScoringL1
          ? DriveToCommands.calculateL1Pose(drive, selection.getAcceptedFaceId())
          : DriveToCommands.calculatePolePose(drive, selection.getAcceptedFaceId(), isLeftPole);

      if (pose == null) {
        System.out.println("Failed to calculate valid target pose.");
      } else {
        System.out.println("Resolved target pose: " + pose);
      }

      return pose;
    };
  }

  public static boolean inScoringConfiguration(CoralSystem coralSystem) {
    return coralSystem.getCurrentCoralPreset() == coralSystem.getTargetCoralPreset()
        && (coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L2
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L3
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L4);
  }
}
