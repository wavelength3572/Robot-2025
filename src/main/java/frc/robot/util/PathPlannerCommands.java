package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.PickupCoralCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.subsystems.coral.CoralSystem;

import static frc.robot.subsystems.coral.CoralSystemPresets.*;

public class PathPlannerCommands {
    public static void Setup(CoralSystem coralSystem)
    {
        NamedCommands.registerCommand("L4", new InstantCommand(()->coralSystem.setTargetPreset(SCORE_LEVEL_4), coralSystem));
        NamedCommands.registerCommand("L3", new InstantCommand(()->coralSystem.setTargetPreset(SCORE_LEVEL_3), coralSystem)); 
        NamedCommands.registerCommand("L2", new InstantCommand(()->coralSystem.setTargetPreset(SCORE_LEVEL_2), coralSystem));
        NamedCommands.registerCommand("L1", new InstantCommand(()->coralSystem.setTargetPreset(SCORE_LEVEL_1), coralSystem));
        NamedCommands.registerCommand("Stow", new InstantCommand(()->coralSystem.setTargetPreset(STOW), coralSystem));
        NamedCommands.registerCommand("Pickup", new InstantCommand(()->coralSystem.setTargetPreset(PICKUP), coralSystem));


        NamedCommands.registerCommand("Score", new ScoreCoralCommand(coralSystem.getIntake()));
        NamedCommands.registerCommand("Intake", new PickupCoralCommand(coralSystem.getIntake()));



    }
}
