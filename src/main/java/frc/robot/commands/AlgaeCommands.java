package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.RobotStatus;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AlgaeCommands {

  private static final double SPEED_SCALAR = 1.0;

  private AlgaeCommands() {
    // Prevent instantiation - utility class
  }

  public static Command AlgaeAlignment(Drive drive, CoralSystem coralSystem, OperatorInterface oi) {

    Command alignmentCommand =
        Commands.defer(
            () ->
                drive.isAtPose(
                        drive.getAlgaeTargetPose(),
                        FieldConstants.ALGAE_ALIGNMENT_TRANSLATION_TOLERANCE,
                        FieldConstants.ALGAE_ALIGNMENT_ANGLE_TOLERANCE_DEGREES)
                    ? Commands.none()
                    : new DriveToPose(drive, drive::getAlgaeTargetPose),
            Set.of(drive));

    Command coralPrepCommand =
        new ConditionalCommand(
            new SequentialCommandGroup(
                SetAppropriateDislodgePresetPart1Command(drive, coralSystem),
                new WaitUntilCommand(coralSystem::isAtGoal),
                SetAppropriateDislodgePresetPart2Command(coralSystem),
                new WaitUntilCommand(coralSystem::isAtGoal)),
            Commands.none(),
            () -> !isCoralPreppedForDislodge());

    Command dislodgeCommand = createDislodgeSequence(drive, coralSystem, oi);

    return new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelCommandGroup(alignmentCommand, coralPrepCommand),
                new WaitUntilCommand(
                    () ->
                        drive.isAtPose(
                                drive.getAlgaeTargetPose(),
                                FieldConstants.ALGAE_ALIGNMENT_TRANSLATION_TOLERANCE,
                                FieldConstants.ALGAE_ALIGNMENT_ANGLE_TOLERANCE_DEGREES)
                            && coralSystem.isAtGoal()
                            && !coralSystem.isHaveCoral()),
                dislodgeCommand),
            Commands.none(),
            () -> {
              AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
              boolean withinThreshold =
                  selection != null
                      && selection.getAcceptedDistance()
                          < FieldConstants.THRESHOLD_DISTANCE_FOR_DISLODGE;
              boolean noCoral = !coralSystem.isHaveCoral();

              return withinThreshold && noCoral;
            })
        .finallyDo(
            (interrupted) -> {
              if (interrupted) {
                Commands.sequence(
                        new WaitUntilCommand(coralSystem::isAtGoal)
                            .withTimeout(2.0), // 2-second timeout for extra safety
                        Commands.runOnce(
                            () -> coralSystem.setTargetPreset(CoralSystemPresets.PICKUP),
                            coralSystem))
                    .schedule();
                Logger.recordOutput(
                    "AlgaeAlignment", "Canceled: waited (with timeout) then reset to STOWED.");
              }
            });
  }

  private static boolean isCoralPreppedForDislodge() {
    CoralSystemPresets targetPreset = RobotStatus.getTargetPreset();
    return targetPreset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1
        || targetPreset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2
        || targetPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
        || targetPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2;
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
                      double offsetMeters = -Units.inchesToMeters(7); // Move backward by 10 inches
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
