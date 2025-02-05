package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
          coralSystem.getElevator().setPositionMeters(preset.getElevatorHeight());
          coralSystem.getArm().setAngleDEG(preset.getArmAngle());
        },
        coralSystem);
  }

  /** Set the Arm Position */
  public static Command setArmPositionFromDashboard(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          coralSystem.getArm().setAngleDEG(SmartDashboard.getNumber("Arm Goal (DEG)", 90.0));
        },
        coralSystem);
  }

  /** Set the Elevator Position */
  public static Command setElevatorPositionFromDashboard(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          coralSystem
              .getElevator()
              .setPositionInches(SmartDashboard.getNumber("Elevator Goal (in)", 0));
        },
        coralSystem);
  }

}
