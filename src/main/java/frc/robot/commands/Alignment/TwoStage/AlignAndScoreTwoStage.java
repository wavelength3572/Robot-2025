package frc.robot.commands.Alignment.TwoStage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return Commands.sequence(
        // INIT stage
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "INIT")),

        // Log reef face, preset, and left/right
        Commands.runOnce(
            () -> {
              Logger.recordOutput(
                  "AlignAndScoreTwoStage/ReefFaceId",
                  drive.getReefFaceSelection().getAcceptedFaceId());
              Logger.recordOutput(
                  "AlignAndScoreTwoStage/CoralPreset", coralSystem.getCurrentCoralPreset().name());
              Logger.recordOutput("AlignAndScoreTwoStage/IsLeftPole", isLeftPole);
            }),

        // ALIGNING stage
        new AlignToReefTwoStage(drive, drive.getReefFaceSelection().getAcceptedFaceId(), isLeftPole)
            .beforeStarting(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "ALIGNING")),

        // CHECK_PRESETS stage
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "CHECK_PRESETS")),

        // WAITING for preset
        Commands.runOnce(
            () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "WAITING_FOR_PRESET")),
        new TimedWaitUntilCommand("AlignAndScoreTwoStage/InPreset", coralSystem::isAtGoal),

        // Log result of scoring condition
        Commands.runOnce(
            () -> {
              boolean willScore =
                  inScoringConfiguration(coralSystem)
                      && drive.getReefFaceSelection().getTagSeenRecently()
                      && ((BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
                                  == BranchAlignmentStatus.GREEN
                              && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.L1_SCORE)
                          || (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_SCORE));

              Logger.recordOutput("AlignAndScoreTwoStage/WillScore", willScore);
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
            // Condition
            () ->
                inScoringConfiguration(coralSystem)
                    && drive.getReefFaceSelection().getTagSeenRecently()
                    && ((BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
                                == BranchAlignmentStatus.GREEN
                            && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.L1_SCORE)
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
