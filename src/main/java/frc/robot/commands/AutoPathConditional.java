package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NamedCommands.RunPresetCommand;
import frc.robot.commands.NamedCommands.ScoreCoralInAutoCommand;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class AutoPathConditional {
  private static final double HIGH_SCORE_EXTRA_TIME_TO_BE_SAFE =
      ScoreCoralInAutoCommand.getExpectedDuration()
          + RunPresetCommand.getExpectedDurationToStowFromL4();
  private static final double LOW_SCORE_EXTRA_TIME_TO_BE_SAFE = 0.0;

  private final String conditionalCommandName;
  private final PathPlannerPath highScorePath;
  private final PathPlannerPath lowScorePath;
  private final PathPlannerPath safePath;
  private final CoralSystem coralSystem;
  private RobotConfig config;

  private double highScoreThresholdTime;
  private double lowScoreThresholdTime;

  public AutoPathConditional(
      String name,
      PathPlannerPath highPath,
      PathPlannerPath lowPath,
      PathPlannerPath safePath,
      CoralSystem coralSystem) {
    this.conditionalCommandName = name;
    this.highScorePath = highPath;
    this.lowScorePath = lowPath;
    this.safePath = safePath;
    this.coralSystem = coralSystem;
  }

  /**
   * Build nested ConditionalCommand: - If enough time for high score path, run highScoreCommand. -
   * Else if enough time for low score path, run lowScoreCommand. - Otherwise, run the safe fallback
   * command.
   */
  public ConditionalCommand getCommand() {
    config = DriveConstants.ppConfig;

    Command highScoreCommand = createHighScoreCommand();
    Command lowScoreCommand = createLowScoreCommand();
    Command fallbackCommand = createSafeCommand();

    highScoreThresholdTime = calcualteHighScoreTimeThreshold();
    lowScoreThresholdTime = calcualteLowScoreTimeThreshold();

    // Create nested conditional for the low score decision.
    // If enough time for low score, run lowScoreCommand; otherwise, safeCommand.
    ConditionalCommand lowScoreConditional =
        new ConditionalCommand(lowScoreCommand, fallbackCommand, this::enoughTimeForLowScore);

    // If enough time for high score, run highScoreCommand; otherwise, evaluate the
    // low score
    // condition.
    ConditionalCommand finalDecisionCommand =
        new ConditionalCommand(highScoreCommand, lowScoreConditional, this::enoughTimeForHighScore);

    Logger.recordOutput("AutoPathConditional/HighScore/CommandName", highScoreCommand.getName());
    Logger.recordOutput("AutoPathConditional/LowScore/CommandName", lowScoreCommand.getName());
    Logger.recordOutput("AutoPathConditional/FallbackCommandName", fallbackCommand.getName());
    Logger.recordOutput("AutoPathConditional/ConditionalCommandName", getConditionalCommandName());
    return finalDecisionCommand;
  }

  private Command createHighScoreCommand() {
    Command highScoreCommand =
        new SequentialCommandGroup(
            AutoBuilder.followPath(highScorePath),
            new TimedWaitUntilCommand("High Score Conditional", coralSystem::isAtGoal),
            new ScoreCoralInAutoCommand(coralSystem.getIntake()),
            new RunPresetCommand(coralSystem, CoralSystemPresets.AUTO_SCORE_END));
    return highScoreCommand;
  }

  private Command createLowScoreCommand() {
    Command lowScoreCommand =
        new SequentialCommandGroup(
            AutoBuilder.followPath(lowScorePath),
            new TimedWaitUntilCommand("Low Score Conditional", coralSystem::isAtGoal),
            new ScoreCoralInAutoCommand(coralSystem.getIntake()),
            new RunPresetCommand(coralSystem, CoralSystemPresets.L1_SCORE));
    return lowScoreCommand;
  }

  private Command createSafeCommand() {
    if (safePath != null) {
      Command safeCommand = AutoBuilder.followPath(safePath);
      return safeCommand;
    } else return Commands.none();
  }

  private double calcualteHighScoreTimeThreshold() {
    PathPlannerTrajectory highScoreTrajectory = highScorePath.getIdealTrajectory(config).get();
    double highScorePathTime = highScoreTrajectory.getTotalTimeSeconds();
    Logger.recordOutput("AutoPathConditional/HighScore/PathTrajectoryTime", highScorePathTime);
    double thresholdTime = highScorePathTime + HIGH_SCORE_EXTRA_TIME_TO_BE_SAFE;
    Logger.recordOutput("AutoPathConditional/HighScore/ThresholdTime", thresholdTime);
    return thresholdTime;
  }

  private double calcualteLowScoreTimeThreshold() {
    PathPlannerTrajectory lowScoreTrajectory = lowScorePath.getIdealTrajectory(config).get();
    double lowScorePathTime = lowScoreTrajectory.getTotalTimeSeconds();
    Logger.recordOutput("AutoPathConditional/LowScore/PathTrajectoryTime", lowScorePathTime);
    double thresholdTime = lowScorePathTime + LOW_SCORE_EXTRA_TIME_TO_BE_SAFE;
    Logger.recordOutput("AutoPathConditional/LowScore/ThresholdTime", thresholdTime);
    return thresholdTime;
  }

  public String getConditionalCommandName() {
    return conditionalCommandName;
  }

  /** Checks whether there is enough match time for the high score path. */
  private boolean enoughTimeForHighScore() {
    double matchTime = DriverStation.getMatchTime();
    Logger.recordOutput("AutoPathConditional/HighScore/MatchTime", matchTime);
    boolean enoughTime = matchTime > highScoreThresholdTime;
    Logger.recordOutput("AutoPathConditional/HighScore/enoughTime", enoughTime);
    return enoughTime;
  }

  /** Checks whether there is enough match time for the low score path. */
  private boolean enoughTimeForLowScore() {
    double matchTime = DriverStation.getMatchTime();
    Logger.recordOutput("AutoPathConditional/LowScore/MatchTime", matchTime);
    boolean enoughTime = matchTime > lowScoreThresholdTime;
    Logger.recordOutput("AutoPathConditional/LowScore/enoughTime", enoughTime);
    return enoughTime;
  }
}
