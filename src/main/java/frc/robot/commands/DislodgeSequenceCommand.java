package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;

public class DislodgeSequenceCommand extends SequentialCommandGroup {

  /**
   * Constructs the dislodge sequence command.
   *
   * @param drive The drive subsystem.
   * @param coralSystem The coral system subsystem.
   * @param oi The operator interface containing the joystick suppliers.
   */
  public DislodgeSequenceCommand(Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    addCommands(
        // 1. Determine and set the final dislodge preset based on the current prepare
        // preset.
        new InstantCommand(
            () -> {
              CoralSystemPresets currentPreset = coralSystem.getCurrentCoralPreset();
              CoralSystemPresets finalPreset;
              if (currentPreset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1) {
                finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_1;
              } else if (currentPreset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2) {
                finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_2;
              } else {
                // Fallback: if not in a valid prepare state, default to one of the finals.
                finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_1;
              }
              coralSystem.setTargetPreset(finalPreset);
            },
            coralSystem),

        // 2. Wait for a fixed delay (e.g., 0.2 seconds) to allow the mechanism to
        // settle.
        new WaitCommand(0.2),

        // 3. Drive backward 10 inches relative to the current pose.
        DriveToCommands.createDriveToPose(
            drive,
            () -> {
              Pose2d currentPose = drive.getPose();
              double offsetMeters = -0.254; // 10 inches backward
              Translation2d offset =
                  new Translation2d(offsetMeters, 0).rotateBy(currentPose.getRotation());
              Translation2d targetTranslation = currentPose.getTranslation().plus(offset);
              return new Pose2d(targetTranslation, currentPose.getRotation());
            },
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate),

        // 4. Finally, command the coral system to move to the PICKUP preset.
        new InstantCommand(
            () -> {
              coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
            },
            coralSystem));
  }
}
