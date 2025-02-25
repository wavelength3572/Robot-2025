package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import org.littletonrobotics.junction.Logger; // Make sure this is the correct Logger import

/**
 * Moves the CoralSystem to a given preset. The command finishes when the subsystem reports it has
 * reached its goal.
 */
public class RunPresetCommand extends Command {
  private static final double EXPECTED_DURATION_TO_STOW_FROM_L4 = 1.0;
  private static final double EXPECTED_DURATION_TO_STOW_FROM_L2 = 0.5;
  private final CoralSystem coralSystem;
  private final CoralSystemPresets targetPreset;

  public RunPresetCommand(CoralSystem coralSystem, CoralSystemPresets targetPreset) {
    this.coralSystem = coralSystem;
    this.targetPreset = targetPreset;
  }

  @Override
  public void initialize() {
    // Set the target preset and log the initialization with an output value of 0.
    coralSystem.setTargetPreset(targetPreset);
    Logger.recordOutput("Commands/RunPresetCommand/Init", targetPreset);
  }

  @Override
  public boolean isFinished() {
    boolean atGoal = coralSystem.isAtGoal();
    // Log the goal status: log 1 if at goal, otherwise log 0.
    Logger.recordOutput("Commands/RunPresetCommand/AtGoal", atGoal);
    return atGoal;
  }

  public static double getExpectedDurationToStowFromL4() {
    return EXPECTED_DURATION_TO_STOW_FROM_L4;
  }

  public static double getExpectedDurationToStowFromL2() {
    return EXPECTED_DURATION_TO_STOW_FROM_L2;
  }
}
