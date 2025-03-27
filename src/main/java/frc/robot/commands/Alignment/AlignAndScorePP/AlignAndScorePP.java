package frc.robot.commands.Alignment.AlignAndScorePP;

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

/** Command that aligns the robot to the pole and scores if the preset is correct. */
public class AlignAndScorePP {

  /**
   * Creates the full command sequence to align the robot to a pole (via AlignToReefPP) and then
   * score if the coral system is in a scoring preset.
   *
   * @param drive The drive subsystem.
   * @param coralSystem The coral system handling the coral manipulator.
   * @param isLeftPole Whether the target pole is on the left side.
   * @return The full command sequence.
   */
  public static Command create(Drive drive, CoralSystem coralSystem, boolean isLeftPole) {
    return Commands.sequence(
        // Stage: INIT
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScorePP/Stage", "INIT")),

        // Run the path-planned alignment
        new AlignToReefPP(drive, isLeftPole)
            .beforeStarting(() -> Logger.recordOutput("AlignAndScorePP/Stage", "ALIGNING")),

        // Post-alignment log
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScorePP/Stage", "CHECK_PRESETS")),

        // Wait until the coral system is at its goal
        new TimedWaitUntilCommand("inPresetConfiguration", coralSystem::isAtGoal),

        // Either auto-score if conditions match or do nothing
        Commands.either(
            // SCORING branch
            Commands.sequence(
                Commands.runOnce(() -> Logger.recordOutput("AlignAndScorePP/Stage", "SCORING")),
                new ScoreCoralInTeleopCommand(coralSystem.getIntake()),
                Commands.runOnce(
                    () -> Logger.recordOutput("AlignAndScorePP/Stage", "SCORING_DONE"))),
            // NO SCORING branch
            Commands.runOnce(() -> Logger.recordOutput("AlignAndScorePP/Stage", "NO_SCORING")),
            // Condition for scoring
            () ->
                inScoringConfiguration(coralSystem)
                    && drive.getReefFaceSelection().getTagSeenRecently()
                    && ((BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
                                == BranchAlignmentStatus.GREEN
                            && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.L1_SCORE)
                        || (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_SCORE))));
  }

  /** Helper method that checks if the coral system is in one of the valid "scoring" presets. */
  public static boolean inScoringConfiguration(CoralSystem coralSystem) {
    return coralSystem.getCurrentCoralPreset() == coralSystem.getTargetCoralPreset()
        && (coralSystem.getTargetCoralPreset() == CoralSystemPresets.L1_SCORE
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L2
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L3
            || coralSystem.getTargetCoralPreset() == CoralSystemPresets.L4);
  }
}
