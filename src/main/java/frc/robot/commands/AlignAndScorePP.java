package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.BranchAlignmentUtils;
import frc.robot.util.BranchAlignmentUtils.BranchAlignmentStatus;

public class AlignAndScorePP {

  /**
   * Command that aligns the robot to the pole and scores if the preset is correct.
   *
   * @param drive The drive subsystem.
   * @param coralSystem The coral handling system.
   * @param isLeftPole Whether the target pole is on the left side.
   * @return The full command sequence.
   */
  public static Command create(Drive drive, CoralSystem coralSystem, boolean isLeftPole) {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting drive to pole...")),

        // Directly call AlignToReef instead of manually checking the pose
        new AlignToReefPP(drive, isLeftPole),
        Commands.runOnce(() -> System.out.println("Robot aligned, wait for preset to finish...")),
        new TimedWaitUntilCommand("inPresetConfiguration", coralSystem::isAtGoal),
        // Auto-score only if preset matches
        Commands.either(
            new ScoreCoralInTeleopCommand(coralSystem.getIntake()),
            Commands.none(),
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
