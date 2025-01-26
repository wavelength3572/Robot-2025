package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;

public class WLButtons {

  private static OperatorInterface oi;
  private static Drive WLDrive;

  public WLButtons() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    WLDrive = drive;
  }

  public static void configureButtonBindings(OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    WLDrive = drive;

    WLDrive.setDefaultCommand(
        DriveCommands.joystickHybridDrive(
            drive,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            drive::getPose,
            drive::getReefFaceSelection,
            2.0));

    configureDriverButtons();
  }

  private static void configureDriverButtons() {

    // Gyro Reset
    oi.getResetGyroButton().onTrue(Commands.runOnce(WLDrive::zeroGyroscope, WLDrive));
    // Angle Drive Button
    oi.getAngleDriveButton()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                WLDrive, oi::getTranslateX, oi::getTranslateY, () -> new Rotation2d()));
    // Drive to Pole
    oi.getRightJoyLeftButton()
        .onTrue(
            DriveToCommands.driveToPole(
                WLDrive,
                true,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                CommandConstants.THRESHOLD_DISTANCE));
    oi.getRightJoyRightButton()
        .onTrue(
            DriveToCommands.driveToPole(
                WLDrive,
                false,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                CommandConstants.THRESHOLD_DISTANCE));
  }
}
