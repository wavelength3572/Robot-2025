package frc.robot.commands;

import static frc.robot.subsystems.coral.CoralSystemPresets.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.NamedCommands.PickupCoralCommand;
import frc.robot.commands.NamedCommands.RunPresetCommand;
import frc.robot.commands.NamedCommands.ScoreCoralCommand;
import frc.robot.subsystems.coral.CoralSystem;

public class PathPlannerCommands {
  public static void Setup(CoralSystem coralSystem) {
    NamedCommands.registerCommand(
        "WaitForCoral", new WaitUntilCommand(coralSystem::isCoralInRobot));
    NamedCommands.registerCommand("WaitForPreset", new WaitUntilCommand(coralSystem::isAtGoal));

    // Register our preset-commands so they finish atGoal
    NamedCommands.registerCommand("L4", new RunPresetCommand(coralSystem, L4));
    NamedCommands.registerCommand("L3", new RunPresetCommand(coralSystem, L3));
    NamedCommands.registerCommand("L2", new RunPresetCommand(coralSystem, L2));
    NamedCommands.registerCommand("L1", new RunPresetCommand(coralSystem, L1));
    NamedCommands.registerCommand("Stow", new RunPresetCommand(coralSystem, STOW));
    NamedCommands.registerCommand("Score", new ScoreCoralCommand(coralSystem.getIntake()));

    new EventTrigger("Pickup")
        .onTrue(Commands.print("Pickup Position").andThen(new PickupCoralCommand(coralSystem)));
    new EventTrigger("L4")
        .onTrue(
            Commands.print("Running preset: L4").andThen(new RunPresetCommand(coralSystem, L4)));
    new EventTrigger("L3")
        .onTrue(
            Commands.print("Running preset: L3").andThen(new RunPresetCommand(coralSystem, L3)));
    new EventTrigger("L2")
        .onTrue(
            Commands.print("Running preset: L2").andThen(new RunPresetCommand(coralSystem, L2)));
    new EventTrigger("L1")
        .onTrue(
            Commands.print("Running preset: L1").andThen(new RunPresetCommand(coralSystem, L1)));
    new EventTrigger("Stow")
        .onTrue(
            Commands.print("Running preset: Stow")
                .andThen(new RunPresetCommand(coralSystem, STOW)));
    new EventTrigger("Score")
        .onTrue(Commands.print("Scoring").andThen(new ScoreCoralCommand(coralSystem.getIntake())));
  }
}
