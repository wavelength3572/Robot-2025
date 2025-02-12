package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import org.littletonrobotics.junction.Logger;

public class AlgaeCommands {

  private AlgaeCommands() {
    // Prevent instantiation - utility class
  }

  /**
   * Creates a command to drive to the dislodge (algae removal) target pose if all conditions are
   * met.
   */
  public static Command driveToDislodge(
      Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    return new ConditionalCommand(
        // True branch: run the sequence if conditions are met.
        new SequentialCommandGroup(
            // 1. Command the coral system to move to STOW.
            new InstantCommand(() -> coralSystem.setTargetPreset(CoralSystemPresets.STOW)),
            // 2. Wait until the system reaches STOW.
            new WaitUntilCommand(coralSystem::isAtGoal),
            // 3. Command the coral system to move to the appropriate dislodge preset based
            // on the current face.
            SetAppropriateDislodgePresetCommand(drive, coralSystem),
            // 4. Wait until the system reaches the dislodge preset.
            new WaitUntilCommand(coralSystem::isAtGoal),
            // 5. Finally, drive to the dislodge (algae removal) target pose.
            DriveToCommands.createDriveToPose(
                drive,
                drive::getAlgaeTargetPose,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate)),
        // False branch: do nothing.
        Commands.none(),
        // Condition: Only execute if:
        // - The coral system is NOT already in a prepare dislodge preset,
        // - The accepted reef face is within the threshold distance,
        // - AND there is NO coral currently in the robot.
        () -> {
          boolean notAtDislodgePreset =
              !(coralSystem.isAtGoal()
                  && (coralSystem.getCurrentCoralPreset()
                          == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1
                      || coralSystem.getCurrentCoralPreset()
                          == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2));

          AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
          boolean withinThreshold = false;
          if (selection != null) {
            double distance = selection.getAcceptedDistance();
            withinThreshold = (distance < FieldConstants.THRESHOLD_DISTANCE_FOR_DISLODGE);
          }

          boolean noCoral = !coralSystem.isCoralInRobot();

          return notAtDislodgePreset && withinThreshold && noCoral;
        });
  }

  /**
   * Creates a command to execute the full dislodge sequence **only if** the robot is in a dislodge
   * preset and there is no coral in the robot.
   */
  public static Command createDislodgeSequence(
      Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    return new ConditionalCommand(
        // ✅ **True Branch**: Execute the dislodge sequence
        new SequentialCommandGroup(
            // 1. Determine and set the final dislodge preset based on the current prepare
            // preset.
            new InstantCommand(
                () -> {
                  CoralSystemPresets currentPreset = coralSystem.getCurrentCoralPreset();
                  CoralSystemPresets finalPreset;
                  if (currentPreset == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1) {
                    finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_1;
                  } else if (currentPreset == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2) {
                    finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_2;
                  } else {
                    System.out.println(
                        "[AlgaeCommands] Warning: Unexpected preset! Defaulting to FINAL_DISLODGE_LEVEL_1.");
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
                  double offsetMeters = -0.254; // Move backward by 10 inches
                  Translation2d offset =
                      new Translation2d(offsetMeters, 0).rotateBy(currentPose.getRotation());
                  Translation2d targetTranslation = currentPose.getTranslation().plus(offset);
                  return new Pose2d(targetTranslation, currentPose.getRotation());
                },
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate),

            // 4. Finally, move the coral system to PICKUP preset.
            new InstantCommand(
                () -> {
                  coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
                },
                coralSystem)),

        // ❌ **False Branch**: Do nothing (command does not run)
        Commands.none(),

        // 🛑 **Condition Check**: Execute only if:
        () -> {
          boolean isInDislodgePreset =
              coralSystem.isAtGoal()
                  && (coralSystem.getCurrentCoralPreset()
                          == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1
                      || coralSystem.getCurrentCoralPreset()
                          == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2);

          boolean noCoral = !coralSystem.isCoralInRobot();

          // Debugging Log
          System.out.println(
              "[AlgaeCommands] Dislodge Check: isInDislodgePreset="
                  + isInDislodgePreset
                  + ", noCoral="
                  + noCoral);

          return isInDislodgePreset && noCoral;
        });
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
            Logger.recordOutput("CoralSystem/DislodgePreset", preset.name());
          }
        },
        coralSystem);
  }
}
