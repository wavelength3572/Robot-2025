package frc.robot.commands;

import static frc.robot.subsystems.coral.CoralSystemPresets.*;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.NamedCommands.PickupCoralCommand;
import frc.robot.commands.NamedCommands.RunPresetCommand;
import frc.robot.commands.NamedCommands.ScoreCoralCommand;
import frc.robot.subsystems.coral.CoralSystem;

public class PathPlannerCommands {
  public static void Setup(CoralSystem coralSystem) {
    // Register our preset-commands so they finish atGoal
    NamedCommands.registerCommand("L4", new RunPresetCommand(coralSystem, SCORE_LEVEL_4));
    NamedCommands.registerCommand("L3", new RunPresetCommand(coralSystem, SCORE_LEVEL_3));
    NamedCommands.registerCommand("L2", new RunPresetCommand(coralSystem, SCORE_LEVEL_2));
    NamedCommands.registerCommand("L1", new RunPresetCommand(coralSystem, SCORE_LEVEL_1));
    NamedCommands.registerCommand("Stow", new RunPresetCommand(coralSystem, STOW));
    NamedCommands.registerCommand("Pickup", new RunPresetCommand(coralSystem, PICKUP));

    NamedCommands.registerCommand("Score", new ScoreCoralCommand(coralSystem.getIntake()));
    NamedCommands.registerCommand("Intake", new PickupCoralCommand(coralSystem.getIntake()));
  }
}
