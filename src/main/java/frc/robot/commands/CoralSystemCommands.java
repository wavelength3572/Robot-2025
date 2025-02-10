package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class CoralSystemCommands {

  private CoralSystemCommands() {}

  public static Command runPreset(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          CoralSystemPresets preset = coralSystem.getCoralSystemPresetChooser().getSelected();
          if (preset != null) {
            coralSystem.setTargetPreset(preset);
            coralSystem.getElevator().setPositionInches(preset.getElevatorHeight());
            coralSystem.getArm().setAngleDEG(preset.getArmAngle());
          }
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

  public static Command SetStowPresetCommand(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          coralSystem.setTargetPreset(CoralSystemPresets.STOW);
          coralSystem.getElevator().setPositionInches(CoralSystemPresets.STOW.getElevatorHeight());
          coralSystem.getArm().setAngleDEG(CoralSystemPresets.STOW.getArmAngle());
        },
        coralSystem);
  }

  public static Command SetAppropriateDislodgePresetCommand(Drive drive, CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          // Get the current reef face selection from the drive subsystem.
          Integer faceId = drive.getReefFaceSelection().getAcceptedFaceId();
          if (faceId == null) {
            return;
          }

          CoralSystemPresets preset = FieldConstants.getDislodgePresetForFace(faceId);
          if (preset != null) {
            // Command the coral system to use the appropriate preset.
            coralSystem.setTargetPreset(preset);
            coralSystem.getElevator().setPositionInches(preset.getElevatorHeight());
            coralSystem.getArm().setAngleDEG(preset.getArmAngle());
            Logger.recordOutput("CoralSystem/DislodgePreset", preset.name());
          }
        },
        coralSystem);
  }
}
