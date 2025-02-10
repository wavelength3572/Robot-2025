package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CoralSystemCommands;
import frc.robot.commands.DislodgeSequenceCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.AlignmentUtils;

public class WLButtons {

  private static OperatorInterface oi;
  private static Drive drive;
  private static CoralSystem coralSystem;
  private static IndicatorLight indicatorLight;

  public WLButtons() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    WLButtons.drive = drive;
  }

  // Updated method signature to receive CoralSubsystem instead of Elevator.
  public static void configureButtonBindings(
      OperatorInterface operatorInterface,
      Drive drive,
      CoralSystem coralSystem,
      IndicatorLight indicatorLight) {
    WLButtons.oi = operatorInterface;
    WLButtons.drive = drive;
    WLButtons.coralSystem = coralSystem;
    WLButtons.indicatorLight = indicatorLight;

    drive.setDriveModeNormal();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    configureDriverButtons();
  }

  private static void configureDriverButtons() {

    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    // Then create the toggle command
    final Command toggleDriveModeCmd =
        DriveCommands.toggleSmartDriveCmd(
            drive,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            coralSystem::isCoralInRobot);

    SmartDashboard.putData("Toggle Smart Drive", toggleDriveModeCmd);

    oi.getRightJoyLeftButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (coralSystem.isCoralInRobot()) {
                    // If there's coral in the robot, drive to a pole
                    DriveToCommands.driveToPole(
                            drive,
                            /* isLeftPole = */ true,
                            oi::getTranslateX,
                            oi::getTranslateY,
                            oi::getRotate,
                            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE)
                        .schedule();
                  }
                },
                drive));

    oi.getRightJoyRightButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (coralSystem.isCoralInRobot()) {
                    // If there's coral in the robot, drive to a pole
                    DriveToCommands.driveToPole(
                            drive,
                            false,
                            oi::getTranslateX,
                            oi::getTranslateY,
                            oi::getRotate,
                            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE)
                        .schedule();
                  }
                },
                drive));

    oi.getLeftJoyLeftButton()
        .onTrue(
            new ConditionalCommand(
                // True branch: run the sequence if conditions are met.
                new SequentialCommandGroup(
                    // 1. Command the coral system to move to STOW.
                    CoralSystemCommands.SetStowPresetCommand(coralSystem),
                    // 2. Wait until the system reaches STOW.
                    new WaitUntilCommand(() -> coralSystem.isAtGoal()),
                    // 3. Command the coral system to move to the appropriate dislodge preset based
                    // on the current face.
                    CoralSystemCommands.SetAppropriateDislodgePresetCommand(drive, coralSystem),
                    // 4. Wait until the system reaches the dislodge preset.
                    new WaitUntilCommand(() -> coralSystem.isAtGoal()),
                    // 5. Finally, drive to the dislodge (algae removal) target pose.
                    DriveToCommands.createDriveToPose(
                        drive,
                        drive::getAlgaeTargetPose,
                        oi::getTranslateX,
                        oi::getTranslateY,
                        oi::getRotate)),
                // False branch: do nothing.
                Commands.none(),
                // Condition: run the sequence only if:
                // - The coral system is not already in a prepare dislodge preset,
                // - The accepted reef face is within the threshold distance,
                // - AND there is no coral currently in the robot.
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
                }));

    oi.getLeftJoyRightButton()
        .onTrue(
            new ConditionalCommand(
                new DislodgeSequenceCommand(drive, coralSystem, oi),
                Commands.none(),
                () ->
                    coralSystem.isAtGoal()
                        && (coralSystem.getCurrentCoralPreset()
                                == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1
                            || coralSystem.getCurrentCoralPreset()
                                == CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2)));

    if (oi.getButtonV().getAsBoolean()) {
      coralSystem.setCoralInRobot(true);
    } else coralSystem.setCoralInRobot(false);

    oi.getButtonV()
        .onTrue(Commands.runOnce(() -> coralSystem.setCoralInRobot(true), coralSystem))
        .onFalse(Commands.runOnce(() -> coralSystem.setCoralInRobot(false), coralSystem));

    oi.getButtonFPosition0() // Push Intake
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralSystem.getIntake().setSpeed(IntakeConstants.intakeOutSpeed);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  coralSystem.getIntake().setSpeed(0.0);
                }));
    oi.getButtonFPosition2() // Pull Intake
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralSystem.getIntake().setSpeed(IntakeConstants.intakeInSpeed);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  coralSystem.getIntake().setSpeed(0.0);
                }));

    if (oi.getButtonGPosition0().getAsBoolean()) {
      AlignmentUtils.setLeftCage();
    } else if (oi.getButtonGPosition1().getAsBoolean()) {
      AlignmentUtils.setMidCage();
    } else {
      AlignmentUtils.setRightCage();
    }

    oi.getButtonGPosition0().onTrue(Commands.runOnce(AlignmentUtils::setLeftCage));
    oi.getButtonGPosition1().onTrue(Commands.runOnce(AlignmentUtils::setMidCage));
    oi.getButtonGPosition2().onTrue(Commands.runOnce(AlignmentUtils::setRightCage));
  }
}
