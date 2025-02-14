package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;

public class CoralSystemCommands {

  private CoralSystemCommands() {}

  public static Command runPreset(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          CoralSystemPresets preset = coralSystem.getCoralSystemPresetChooser().getSelected();
          if (preset != null) {
            coralSystem.setTargetPreset(preset);
          }
        },
        coralSystem);
  }



  
}
