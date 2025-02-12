package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static CoralSystem coralSystem;
  private static IndicatorLight indicatorLight;

  public ButtonsAndDashboardBindings() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
  }

  // Updated method signature to receive CoralSubsystem instead of Elevator.
  public static void configureBindings(
      OperatorInterface operatorInterface,
      Drive drive,
      CoralSystem coralSystem,
      IndicatorLight indicatorLight) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.coralSystem = coralSystem;
    ButtonsAndDashboardBindings.indicatorLight = indicatorLight;

    drive.setDriveModeNormal();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    configureDriverButtonBindings();
    configureDashboardBindings();
  }

  private static void configureDashboardBindings() {

    SmartDashboard.putData(
        "Toggle Smart Drive",
        DriveCommands.toggleSmartDriveCmd(
            drive,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            coralSystem::isCoralInRobot));

    SmartDashboard.putData(
        "Toggle Vision", Commands.runOnce(drive::toggleVision).ignoringDisable(true));

    SmartDashboard.putData(
        "Closest A Pole",
        DriveToCommands.driveToPole(
            drive,
            true,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
            coralSystem::isCoralInRobot));

    SmartDashboard.putData(
        "Closest B Pole",
        DriveToCommands.driveToPole(
            drive,
            false,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
            coralSystem::isCoralInRobot));

    SmartDashboard.putData(
        "Algae Prepare to Dislodge", AlgaeCommands.driveToDislodge(drive, coralSystem, oi));
    SmartDashboard.putData(
        "Alage Dislodge", AlgaeCommands.createDislodgeSequence(drive, coralSystem, oi));
  }

  private static void configureDriverButtonBindings() {
    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    // Simulate Coral in Robot
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
                  coralSystem.getIntake().pushCoral();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  coralSystem.getIntake().stopIntake();
                  ;
                }));
    oi.getButtonFPosition2() // Pull Intake
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralSystem.getIntake().pullCoral();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  coralSystem.getIntake().stopIntake();
                  ;
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
