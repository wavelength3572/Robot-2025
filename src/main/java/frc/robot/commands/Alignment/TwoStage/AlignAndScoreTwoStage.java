package frc.robot.commands.Alignment.TwoStage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.commands.ScoreCoralInTeleopCommand;
import frc.robot.commands.TimedWaitUntilCommand;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.BranchAlignmentUtils;
import frc.robot.util.BranchAlignmentUtils.BranchAlignmentStatus;
import org.littletonrobotics.junction.Logger;

public class AlignAndScoreTwoStage {

  public static Command create(Drive drive, CoralSystem coralSystem, boolean isLeftPole) {
    // Retrieve the accepted distance once.
    double acceptedDistance = drive.getReefFaceSelection().getAcceptedDistance();

    // If the face is too far to align, skip the alignment and scoring
    if (acceptedDistance > FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE) {
      Logger.recordOutput(
          "AlignAndScoreTwoStage/alignmentProximityStatus",
          "FACE_TOO_FAR, skipping alignment and scoring.");
      return Commands.none();
    } else {
      // Update the status dynamically based on the distance condition
      Logger.recordOutput(
          "AlignAndScoreTwoStage/alignmentProximityStatus",
          "FACE_WITHIN_RANGE, proceeding with alignment.");
    }

    // If within threshold, build the full alignment and scoring sequence.
    return Commands.sequence(
        // INIT stage
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "INIT")),

        // Log reef face, preset, and left/right information.
        Commands.runOnce(
            () -> {
              Logger.recordOutput(
                  "AlignAndScoreTwoStage/ReefFaceId",
                  drive.getReefFaceSelection().getAcceptedFaceId());
              Logger.recordOutput(
                  "AlignAndScoreTwoStage/CoralPreset", coralSystem.getCurrentCoralPreset());
              Logger.recordOutput("AlignAndScoreTwoStage/IsLeftPole", isLeftPole);
            }),

        // ALIGNING stage
        new AlignToReefTwoStage(
                drive, coralSystem, drive.getReefFaceSelection().getAcceptedFaceId(), isLeftPole)
            .beforeStarting(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "ALIGNING")),

        // CHECK_PRESETS stage
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "CHECK_PRESETS")),

        // WAITING for preset
        Commands.runOnce(
            () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "WAITING_FOR_PRESET")),
        new TimedWaitUntilCommand("AlignAndScoreTwoStage/InPreset", coralSystem::isAtGoal),
        Commands.runOnce(
            () -> {
              boolean inConfig = inScoringConfiguration(coralSystem);
              boolean tagSeen = drive.getReefFaceSelection().getTagSeenRecently();
              boolean isL1 = coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_SCORE;
              boolean alignmentGreen =
                  BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
                      == BranchAlignmentStatus.GREEN;
              boolean inTOFRange =
                  coralSystem.getTimeOfFlightRangeReef() < CoralSystem.inRangeThreshold.get();

              boolean alignmentOk = (alignmentGreen && !isL1 && inTOFRange) || isL1;

              boolean willScore = inConfig && tagSeen && alignmentOk;

              // Log detailed diagnostics
              Logger.recordOutput("AlignAndScoreTwoStage/WillScore", willScore);
              Logger.recordOutput(
                  "AlignAndScoreTwoStage/WillScore/inScoringConfiguration", inConfig);
              Logger.recordOutput("AlignAndScoreTwoStage/WillScore/tagSeenRecently", tagSeen);
              Logger.recordOutput("AlignAndScoreTwoStage/WillScore/currentPresetIsL1", isL1);
              Logger.recordOutput(
                  "AlignAndScoreTwoStage/WillScore/branchAlignmentGreen", alignmentGreen);
              Logger.recordOutput("AlignAndScoreTwoStage/WillScore/inTOFRange", inTOFRange);
              Logger.recordOutput("AlignAndScoreTwoStage/WillScore/alignmentOk", alignmentOk);

              // Optional: human-readable status
              String reason =
                  !inConfig
                      ? "Not in scoring config"
                      : !tagSeen
                          ? "Tag not seen recently"
                          : !alignmentOk
                              ? (isL1
                                  ? "Unknown issue with L1 scoring"
                                  : (!alignmentGreen ? "Alignment not green" : "TOF out of range"))
                              : "Eligible to score";
              Logger.recordOutput("AlignAndScoreTwoStage/WillScore/Reason", reason);
            }),

        // SCORING (or not)
        Commands.either(
            // SCORING branch
            Commands.sequence(
                Commands.runOnce(
                    () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "SCORING")),
                new ScoreCoralInTeleopCommand(coralSystem.getIntake()),
                Commands.runOnce(
                    () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "SCORING_DONE"))),
            // NO SCORING branch
            Commands.runOnce(
                () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "NO_SCORING")),
            // Condition: should we score?
            () ->
                inScoringConfiguration(coralSystem)
                    && drive.getReefFaceSelection().getTagSeenRecently()
                    && ((BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
                                == BranchAlignmentStatus.GREEN
                            && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.L1_SCORE
                            && coralSystem.getTimeOfFlightRangeReef()
                                < CoralSystem.inRangeThreshold.get())
                        || (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_SCORE))));
  }

  public static boolean inScoringConfiguration(CoralSystem coralSystem) {
    return coralSystem.getCurrentCoralPreset() == coralSystem.getTargetCoralPreset()
        && (coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L2
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L3
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L4);
  }
}
