package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;

public class WLButtons {

  private static OperatorInterface oi;
  private static Drive WLDrive;
  private static CoralSystem coralSystem;

  public WLButtons() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    WLDrive = drive;
  }

  // Updated method signature to receive CoralSubsystem instead of Elevator.
  public static void configureButtonBindings(
      OperatorInterface operatorInterface, Drive drive, CoralSystem coralSystem) {
    oi = operatorInterface;
    WLDrive = drive;
    WLButtons.coralSystem = coralSystem;

    // Example of configuring drive defaults using WLDrive and coralSubsystem (if
    // needed):
    if (oi.getButtonH().getAsBoolean()) {
      WLDrive.setDriveModeSmart();
      WLDrive.setDefaultCommand(
          DriveCommands.joystickSmartDrive(
              drive,
              oi::getTranslateX,
              oi::getTranslateY,
              oi::getRotate,
              drive::getPose,
              drive::getReefFaceSelection,
              FieldConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_REEF,
              drive::getCoralStationSelection,
              FieldConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_STATION,
              drive::getCageSelection,
              FieldConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_CAGE,
              // Use the elevator from the coral subsystem
              () -> coralSystem.isCoralInRobot()));
    } else {
      WLDrive.setDriveModeNormal();
      WLDrive.setDefaultCommand(
          DriveCommands.joystickDrive(
              WLDrive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));
    }

    configureDriverButtons();
  }

  private static void configureDriverButtons() {

    // Gyro Reset
    oi.getResetGyroButton().onTrue(Commands.runOnce(WLDrive::zeroGyroscope, WLDrive));

    oi.getButtonH()
        .onFalse(
            Commands.runOnce(
                () -> {
                  WLDrive.setDriveModeNormal();
                  WLDrive.setDefaultCommand(
                      DriveCommands.joystickDrive(
                          WLDrive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));
                },
                WLDrive))
        .onTrue(
            Commands.runOnce(
                () -> {
                  WLDrive.setDriveModeSmart();
                  WLDrive.setDefaultCommand(
                      DriveCommands.joystickSmartDrive(
                          WLDrive,
                          oi::getTranslateX,
                          oi::getTranslateY,
                          oi::getRotate,
                          WLDrive::getPose,
                          WLDrive::getReefFaceSelection,
                          FieldConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_REEF,
                          WLDrive::getCoralStationSelection,
                          FieldConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_STATION,
                          WLDrive::getCageSelection,
                          FieldConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_CAGE,
                          () -> coralSystem.isCoralInRobot()));
                },
                WLDrive));

    // Drive to Pole
    oi.getRightJoyLeftButton()
        .onTrue(
            DriveToCommands.driveToPole(
                WLDrive,
                true,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    oi.getRightJoyRightButton()
        .onTrue(
            DriveToCommands.driveToPole(
                WLDrive,
                false,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    if (oi.getButtonV().getAsBoolean()) {
      coralSystem.setCoralInRobot(true);
    } else coralSystem.setCoralInRobot(false);

    oi.getButtonV()
        .onTrue(Commands.runOnce(() -> coralSystem.setCoralInRobot(true), coralSystem))
        .onFalse(Commands.runOnce(() -> coralSystem.setCoralInRobot(false), coralSystem));

    if (oi.getButtonFPosition0().getAsBoolean()) {
      AlignmentUtils.setLeftCage();
    } else if (oi.getButtonFPosition1().getAsBoolean()) {
      AlignmentUtils.setMidCage();
    } else {
      AlignmentUtils.setRightCage();
    }

    oi.getButtonFPosition0().onTrue(Commands.runOnce(AlignmentUtils::setLeftCage));
    oi.getButtonFPosition1().onTrue(Commands.runOnce(AlignmentUtils::setMidCage));
    oi.getButtonFPosition2().onTrue(Commands.runOnce(AlignmentUtils::setRightCage));
  }
}
