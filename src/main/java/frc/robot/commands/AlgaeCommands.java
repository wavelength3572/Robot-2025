package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.commands.Alignment.TwoStage.AlignToReefTwoStage;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.RobotStatus;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AlgaeCommands {

  private static final double SPEED_SCALAR = 0.5;

  private AlgaeCommands() {
    // Prevent instantiation - utility class
  }

  /**
   * Creates a command to drive to the dislodge (algae removal) target pose if all conditions are
   * met.
   */
  public static Command AlgaeAlignment(Drive drive, CoralSystem coralSystem, OperatorInterface oi) {

    Command alignmentCommand =
        Commands.defer(
            () -> {
              Integer faceId =
                  (drive.getReefFaceSelection() != null)
                      ? drive.getReefFaceSelection().getAcceptedFaceId()
                      : null;
              if (faceId != null) {
                return new AlignToReefTwoStage(drive, coralSystem, faceId, false);
              } else {
                return Commands.none();
              }
            },
            Set.of(drive));

    return new ConditionalCommand(
        new ParallelCommandGroup(
            // Get the Elevator and Arm prepped for dislodge
            new SequentialCommandGroup(
                SetAppropriateDislodgePresetPart1Command(drive, coralSystem),
                new WaitUntilCommand(coralSystem::isAtGoal),
                SetAppropriateDislodgePresetPart2Command(coralSystem),
                new WaitUntilCommand(coralSystem::isAtGoal)),
            // Drive to the dislodge spot simultaneously
            alignmentCommand),
        Commands.none(),
        // Condition: Only execute if:
        // - The accepted reef face is within the threshold distance,
        // - AND there is NO coral currently in the robot.
        () -> {
          AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
          boolean withinThreshold = false;
          if (selection != null) {
            double distance = selection.getAcceptedDistance();
            withinThreshold = (distance < FieldConstants.THRESHOLD_DISTANCE_FOR_DISLODGE);
          }
          boolean noCoral = !coralSystem.isHaveCoral();

          boolean alreadyPreppedForDislodge =
              (RobotStatus.getTargetPreset() == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1
                  || RobotStatus.getTargetPreset()
                      == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2
                  || RobotStatus.getTargetPreset()
                      == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                  || RobotStatus.getTargetPreset()
                      == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2);

          return withinThreshold && noCoral && !alreadyPreppedForDislodge;
        });
  }

  private static Command SetAppropriateDislodgePresetPart2Command(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          if (coralSystem.getCurrentCoralPreset()
              == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1) {
            coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1);
          } else if (coralSystem.getCurrentCoralPreset()
              == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2) {
            coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2);
          }
        },
        coralSystem);
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
                new InstantCommand(coralSystem.getIntake()::pushCoral, coralSystem),
                // 1. Determine and set the final dislodge preset based on the current prepare
                // preset.
                new InstantCommand(
                    () -> {
                      CoralSystemPresets currentPreset = coralSystem.getCurrentCoralPreset();
                      CoralSystemPresets finalPreset;
                      if (currentPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1) {
                        finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_1;
                      } else if (currentPreset
                          == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2) {
                        finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_2;
                      } else {
                        // This branch should never happen because of our outer condition.
                        // If it does, log the problem and return early without setting any final
                        // preset.
                        System.out.println(
                            "[AlgaeCommands] Dislodge sequence canceled: Not in a valid prepare preset.");
                        return;
                      }
                      coralSystem.setTargetPreset(finalPreset);
                    },
                    coralSystem),

                // 3. Drive backward 10 inches relative to the current pose.
                new DriveToPose(
                    drive,
                    () -> {
                      Pose2d currentPose = drive.getPose();
                      double offsetMeters = -0.254 * 3.0; // Move backward by 10 inches
                      Translation2d offset =
                          new Translation2d(offsetMeters, 0).rotateBy(currentPose.getRotation());
                      Translation2d targetTranslation = currentPose.getTranslation().plus(offset);
                      return new Pose2d(targetTranslation, currentPose.getRotation());
                    },
                    SPEED_SCALAR),
                new InstantCommand(coralSystem.getIntake()::stopIntake, coralSystem)),

            // **False Branch**: Do nothing (command does not run)
            Commands.none(),

            // **Condition Check**: Execute only if:
            () -> {
              boolean isInDislodgePreset =
                  coralSystem.isAtGoal()
                      && (coralSystem.getCurrentCoralPreset()
                              == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                          || coralSystem.getCurrentCoralPreset()
                              == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2);

              boolean noCoral = !coralSystem.isHaveCoral();
              return isInDislodgePreset && noCoral;
            })
        .withTimeout(2.5);
  }

  /**
   * Creates a command to execute the full dislodge sequence **only if** the robot is in a dislodge
   * preset and there is no coral in the robot.
   */
  public static Command createAutonomousDislodgeSequence(Drive drive, CoralSystem coralSystem) {
    return new ConditionalCommand(
        new SequentialCommandGroup(
            new InstantCommand(coralSystem.getIntake()::pushCoral),
            new InstantCommand(
                () -> {
                  CoralSystemPresets currentPreset = coralSystem.getCurrentCoralPreset();
                  CoralSystemPresets finalPreset;
                  if (currentPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1) {
                    finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_1;
                  } else if (currentPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2) {
                    finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_2;
                  } else {
                    // This branch should never happen because of our outer condition.
                    // If it does, log the problem and return early without setting any final
                    // preset.
                    System.out.println(
                        "[AlgaeCommands] Dislodge sequence canceled: Not in a valid prepare preset.");
                    return;
                  }
                  coralSystem.setTargetPreset(finalPreset);
                },
                coralSystem),

            // 3. Drive backward 10 inches relative to the current pose.
            new DriveToPose(
                drive,
                () -> {
                  Pose2d currentPose = drive.getPose();
                  double offsetMeters = -0.254 * 3.0; // Move backward by 10 inches
                  Translation2d offset =
                      new Translation2d(offsetMeters, 0).rotateBy(currentPose.getRotation());
                  Translation2d targetTranslation = currentPose.getTranslation().plus(offset);
                  return new Pose2d(targetTranslation, currentPose.getRotation());
                },
                SPEED_SCALAR),
            new InstantCommand(coralSystem.getIntake()::stopIntake)),
        Commands.none(),

        // **Condition Check**: Execute only if:
        () -> {
          boolean isInDislodgePreset =
              coralSystem.isAtGoal()
                  && (coralSystem.getCurrentCoralPreset()
                          == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                      || coralSystem.getCurrentCoralPreset()
                          == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2);

          boolean noCoral = !coralSystem.isHaveCoral();
          return isInDislodgePreset && noCoral;
        });
  }

  public static Command SetAppropriateDislodgePresetPart1Command(
      Drive drive, CoralSystem coralSystem) {
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
