package frc.robot.commands;

import static frc.robot.subsystems.coral.CoralSystemPresets.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.NamedCommands.PickupCoralCommand;
import frc.robot.commands.NamedCommands.RunPresetCommand;
import frc.robot.commands.NamedCommands.ScoreCoralInAutoCommand;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PathPlannerCommands {
  public static void Setup(CoralSystem coralSystem, Drive drive, Vision vision, Algae algae) {
    NamedCommands.registerCommand(
        "WaitForCoral", new TimedWaitUntilCommand("WaitForCoral", coralSystem::isHaveCoral));
    NamedCommands.registerCommand(
        "WaitForPreset", new TimedWaitUntilCommand("WaitForPreset", coralSystem::isAtGoal));
    NamedCommands.registerCommand("CaptureAlgae", Commands.runOnce(algae::pullAlgae));

    NamedCommands.registerCommand("TurnVisionOff", new InstantCommand(vision::setVisionOff));

    NamedCommands.registerCommand("L4", new RunPresetCommand(coralSystem, L4));
    NamedCommands.registerCommand("L3", new RunPresetCommand(coralSystem, L3));
    NamedCommands.registerCommand("L2", new RunPresetCommand(coralSystem, L2));
    NamedCommands.registerCommand("L1", new RunPresetCommand(coralSystem, L1));
    NamedCommands.registerCommand("Stow", new RunPresetCommand(coralSystem, STOW));
    NamedCommands.registerCommand("Score", new ScoreCoralInAutoCommand(coralSystem.getIntake()));
    NamedCommands.registerCommand(
        "DislodgeLow",
        new ParallelCommandGroup(
            new InstantCommand(coralSystem.getIntake()::pushCoral),
            new InstantCommand(
                () ->
                    coralSystem.setSimultaneousTargetPreset(
                        CoralSystemPresets.FINAL_DISLODGE_LEVEL_1))));

    NamedCommands.registerCommand(
        "DislodgeHigh",
        new ParallelCommandGroup(
            new InstantCommand(coralSystem.getIntake()::pushCoral),
            new InstantCommand(
                () ->
                    coralSystem.setSimultaneousTargetPreset(
                        CoralSystemPresets.FINAL_DISLODGE_LEVEL_2))));

    NamedCommands.registerCommand(
        "PrepareLowDislodge",
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1)),
            new WaitUntilCommand(coralSystem::isAtGoal),
            new InstantCommand(
                () ->
                    coralSystem.setSimultaneousTargetPreset(
                        CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1)),
            new WaitUntilCommand(coralSystem::isAtGoal)));

    NamedCommands.registerCommand(
        "PrepareHighDislodge",
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2)),
            new WaitUntilCommand(coralSystem::isAtGoal),
            new InstantCommand(
                () ->
                    coralSystem.setSimultaneousTargetPreset(
                        CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2)),
            new WaitUntilCommand(coralSystem::isAtGoal)));

    new EventTrigger("Pickup")
        .onTrue(Commands.print("Pickup Position").andThen(new PickupCoralCommand(coralSystem)));
    new EventTrigger("L4")
        .onTrue(
            Commands.print("Running preset: L4").andThen(new RunPresetCommand(coralSystem, L4)));
    new EventTrigger("PREL4")
        .onTrue(
            Commands.print("Running preset: PREL4")
                .andThen(new RunPresetCommand(coralSystem, PREL4)));

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
        .onTrue(
            Commands.print("Scoring")
                .andThen(new ScoreCoralInAutoCommand(coralSystem.getIntake())));

    try {
      List<AutoPathConditional> autoPathConditionals = new ArrayList<>();

      autoPathConditionals.add(
          new AutoPathConditional(
              "StationRight-6BHigh-OR-1BLow",
              PathPlannerPath.fromPathFile("StationRight-6BHigh"),
              PathPlannerPath.fromPathFile("StationRight-1BLow"),
              PathPlannerPath.fromPathFile("StationRight-Safe"),
              coralSystem));

      autoPathConditionals.add(
          new AutoPathConditional(
              "Score3-StationRight-6BHigh-OR-1BLow",
              PathPlannerPath.fromPathFile("Score3-StationRight-6BHigh"),
              PathPlannerPath.fromPathFile("Score3-StationRight-1BLow"),
              PathPlannerPath.fromPathFile("StationRight-Safe"),
              coralSystem));

      autoPathConditionals.add( // this is part of our Score2 for LakeCity
          new AutoPathConditional(
              "StationLeft-2BHigh-OR-1ALow",
              PathPlannerPath.fromPathFile("StationLeft-2BHigh"),
              PathPlannerPath.fromPathFile("StationLeft-1ALow"),
              PathPlannerPath.fromPathFile("StationLeft-Safe"),
              coralSystem));

      autoPathConditionals.add( // SCORE 3
          new AutoPathConditional(
              "Score3-StationLeft-2BHigh-OR-1ALow",
              PathPlannerPath.fromPathFile("Score3-StationLeft-2BHigh"),
              PathPlannerPath.fromPathFile("Score3-StationLeft-1ALow"),
              PathPlannerPath.fromPathFile("StationLeft-Safe"),
              coralSystem));

      // Register each combination as a NamedCommand
      for (AutoPathConditional conditional : autoPathConditionals) {
        NamedCommands.registerCommand(
            conditional.getConditionalCommandName(), conditional.getCommand());
        Logger.recordOutput(
            "ConditionalCommands/" + conditional.getConditionalCommandName(),
            conditional.getCommand().getName());
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }
}
