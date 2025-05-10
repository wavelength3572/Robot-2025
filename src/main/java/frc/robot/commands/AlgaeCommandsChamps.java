package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.FieldConstants;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.RobotStatus;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AlgaeCommandsChamps {

  private AlgaeCommandsChamps() {}

  public static Command AlgaeAlignmentAndDislodge(
      Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    return new ConditionalCommand(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                SetAppropriateDislodgePresetPart1Command(coralSystem),
                new DriveToPose(drive, drive::getAlgaeTargetPose).withTimeout(1.8)),
            new TimedWaitUntilCommand(
                "WaitForDislodgePart1Stable",
                isAtPreset(
                    coralSystem,
                    CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1,
                    CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2)),
            SetAppropriateDislodgePresetPart2Command(coralSystem),
            new InstantCommand(coralSystem.getIntake()::pushCoral, coralSystem),
            new TimedWaitUntilCommand(
                "WaitForDislodgePart2Stable",
                isAtPreset(
                    coralSystem,
                    CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1,
                    CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2)),
            maybeRunDislodgeSequence(drive, coralSystem, oi)),
        Commands.none(),
        () -> {
          AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
          return selection != null
              && selection.getAcceptedDistance() < FieldConstants.THRESHOLD_DISTANCE_FOR_DISLODGE
              && !coralSystem.isHaveCoral();
        });
  }

  private static BooleanSupplier isAtPreset(
      CoralSystem coralSystem, CoralSystemPresets... presets) {
    return () ->
        coralSystem.isAtGoal()
            && java.util.Arrays.asList(presets).contains(coralSystem.getCurrentCoralPreset());
  }

  private static Command maybeRunDislodgeSequence(
      Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    return new ConditionalCommand(
        createDislodgeSequence(drive, coralSystem, oi),
        Commands.none(),
        () -> {
          CoralSystemPresets preset = coralSystem.getCurrentCoralPreset();
          boolean ready =
              preset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                  || preset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2;
          return coralSystem.isAtGoal() && !coralSystem.isHaveCoral() && ready;
        });
  }

  public static Command createDislodgeSequence(
      Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    return new SequentialCommandGroup(
            new InstantCommand(coralSystem.getIntake()::pushCoral, coralSystem),
            new InstantCommand(
                () -> {
                  CoralSystemPresets current = coralSystem.getCurrentCoralPreset();
                  CoralSystemPresets finalPreset;
                  if (current == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1) {
                    finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_1;
                  } else if (current == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2) {
                    finalPreset = CoralSystemPresets.FINAL_DISLODGE_LEVEL_2;
                  } else {
                    System.out.println(
                        "[AlgaeCommands] Dislodge sequence canceled: Not in a valid prepare preset.");
                    return;
                  }
                  coralSystem.setTargetPreset(finalPreset);
                },
                coralSystem),
            new DriveToPoseJoystickCancel(
                drive,
                () -> {
                  Pose2d currentPose = drive.getPose();
                  Translation2d offset =
                      new Translation2d(-Units.inchesToMeters(10), 0)
                          .rotateBy(currentPose.getRotation());
                  return new Pose2d(
                      currentPose.getTranslation().plus(offset), currentPose.getRotation());
                },
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate),
            new InstantCommand(coralSystem.getIntake()::stopIntake, coralSystem))
        .withTimeout(2.5);
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

  public static Command SetAppropriateDislodgePresetPart1Command(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          AlignmentUtils.ReefFaceSelection selection = RobotStatus.getReefFaceSelection();
          if (selection == null) return;
          Integer faceId = selection.getAcceptedFaceId();
          if (faceId == null) return;

          CoralSystemPresets preset = FieldConstants.getDislodgePresetForFace(faceId);
          if (preset != null) {
            coralSystem.setTargetPreset(preset);
            Logger.recordOutput("CoralSystem/DislodgePreset", preset.name());
          }
        },
        coralSystem);
  }

  public static Command operatorInitiatedDislodge(
      Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    return new SequentialCommandGroup(
        new TimedWaitUntilCommand(
            "WaitForDislodgePart2Stable",
            () -> {
              CoralSystemPresets preset = coralSystem.getCurrentCoralPreset();
              boolean ready =
                  preset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                      || preset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2;
              return coralSystem.isAtGoal() && !coralSystem.isHaveCoral() && ready;
            }),
        createDislodgeSequence(drive, coralSystem, oi));
  }

  public static Command operatorInitiatedPart2PlusDislodge(
      Drive drive, CoralSystem coralSystem, OperatorInterface oi) {
    return new SequentialCommandGroup(
        new TimedWaitUntilCommand(
            "WaitForDislodgePart1Stable",
            () -> {
              CoralSystemPresets preset = coralSystem.getCurrentCoralPreset();
              boolean ready =
                  preset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1
                      || preset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2;
              return coralSystem.isAtGoal() && !coralSystem.isHaveCoral() && ready;
            }),
        SetAppropriateDislodgePresetPart2Command(coralSystem),
        new TimedWaitUntilCommand(
            "WaitForDislodgePart2Stable",
            isAtPreset(
                coralSystem,
                CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1,
                CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2)),
        createDislodgeSequence(drive, coralSystem, oi));
  }
}
