package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.AlignmentUtils;

public class WLButtons {

  private static OperatorInterface oi;
  private static Drive WLDrive;
  private static CoralSystem coralSystem;
  private static IndicatorLight WLIndicatorLight;

  public WLButtons() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    WLDrive = drive;
  }

  // Updated method signature to receive CoralSubsystem instead of Elevator.
  public static void configureButtonBindings(
      OperatorInterface operatorInterface,
      Drive drive,
      CoralSystem coralSystem,
      IndicatorLight indicatorLight) {
    oi = operatorInterface;
    WLDrive = drive;
    WLButtons.coralSystem = coralSystem;
    WLIndicatorLight = indicatorLight;

    WLDrive.setDriveModeNormal();
    WLDrive.setDefaultCommand(
        DriveCommands.joystickDrive(WLDrive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    configureDriverButtons();
  }

  private static void configureDriverButtons() {

    // Gyro Reset
    oi.getResetGyroButton().onTrue(Commands.runOnce(WLDrive::zeroGyroscope, WLDrive));

    // Then create the toggle command
    final Command toggleDriveModeCmd =
        DriveCommands.toggleSmartDriveCmd(
            WLDrive,
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
                            WLDrive,
                            /* isLeftPole = */ true,
                            oi::getTranslateX,
                            oi::getTranslateY,
                            oi::getRotate,
                            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE)
                        .schedule();
                  }
                },
                WLDrive));

    oi.getRightJoyRightButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (coralSystem.isCoralInRobot()) {
                    // If there's coral in the robot, drive to a pole
                    DriveToCommands.driveToPole(
                            WLDrive,
                            false,
                            oi::getTranslateX,
                            oi::getTranslateY,
                            oi::getRotate,
                            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE)
                        .schedule();
                  }
                },
                WLDrive));

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
