package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;

/**
 * Moves the CoralSystem to a given preset.
 * The command finishes when the subsystem reports it has reached its goal.
 */
public class RunPresetCommand extends Command {
  private final CoralSystem coralSystem;
  private final CoralSystemPresets targetPreset;

  public RunPresetCommand(CoralSystem coralSystem, CoralSystemPresets targetPreset) {
    this.coralSystem = coralSystem;
    this.targetPreset = targetPreset;
    addRequirements(coralSystem);
  }

  @Override
  public void initialize() {
    coralSystem.setTargetPreset(targetPreset);
  }

  @Override
  public boolean isFinished() {
    // Command will end once the subsystem is at its goal
    return coralSystem.isAtGoal();
  }
}
