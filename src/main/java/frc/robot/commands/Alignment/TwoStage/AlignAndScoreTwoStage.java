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
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "INIT")),
        new AlignToReefTwoStage(drive, drive.getReefFaceSelection().getAcceptedFaceId(), isLeftPole)
            .beforeStarting(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "ALIGNING")),
        Commands.runOnce(() -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "CHECK_PRESETS")),
        new TimedWaitUntilCommand("inPresetConfiguration", coralSystem::isAtGoal),
        Commands.either(
            Commands.sequence(
                Commands.runOnce(
                    () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "SCORING")),
                new ScoreCoralInTeleopCommand(coralSystem.getIntake()),
                Commands.runOnce(
                    () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "SCORING_DONE"))),
            Commands.runOnce(
                () -> Logger.recordOutput("AlignAndScoreTwoStage/Stage", "NO_SCORING")),
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
