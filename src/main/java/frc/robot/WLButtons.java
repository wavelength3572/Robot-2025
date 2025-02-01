package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.AlignmentUtils;

public class WLButtons {

  private static OperatorInterface oi;
  private static Drive WLDrive;
  private static Elevator WLElevator;

  public WLButtons() {
  }

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    WLDrive = drive;
  }

  public static void configureButtonBindings(
      OperatorInterface operatorInterface, Drive drive, Elevator elevator) {
    oi = operatorInterface;
    WLDrive = drive;
    WLElevator = elevator;

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
              CommandConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_REEF,
              drive::getCoralStationSelection,
              CommandConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_STATION,
              elevator::hasCoral));

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
                WLDrive // optional requirement
            ))
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
                          CommandConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_REEF,
                          WLDrive::getCoralStationSelection,
                          CommandConstants.THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_STATION,
                          WLElevator::hasCoral));
                },
                WLDrive // optional requirement
            ));

    // Drive to Pole
    oi.getRightJoyLeftButton()
        .onTrue(
            DriveToCommands.driveToPole(
                WLDrive,
                true,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                CommandConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    oi.getRightJoyRightButton()
        .onTrue(
            DriveToCommands.driveToPole(
                WLDrive,
                false,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                CommandConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    if (oi.getButtonV().getAsBoolean()) {
      WLElevator.setHasCoral();
    } else
      WLElevator.clearHasCoral();

    oi.getButtonV()
        .onTrue(Commands.runOnce(WLElevator::setHasCoral, WLElevator))
        .onFalse(Commands.runOnce(WLElevator::clearHasCoral, WLElevator));

    if (oi.getButtonFPosition0().getAsBoolean()) {
      AlignmentUtils.setLeftCage();
    } else if (oi.getButtonFPosition1().getAsBoolean()) {
      AlignmentUtils.setMidCage();
    } else
      AlignmentUtils.setRightCage();

    oi.getButtonFPosition0()
        .onTrue(Commands.runOnce(AlignmentUtils::setLeftCage));
    oi.getButtonFPosition1()
        .onTrue(Commands.runOnce(AlignmentUtils::setMidCage));
    oi.getButtonFPosition2()
        .onTrue(Commands.runOnce(AlignmentUtils::setRightCage));

    
  }
}
