package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
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
        DriveCommands.joystickDrive(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    configureDriverButtons();
  }

  private static void configureDriverButtons() {

    // Gyro Reset
    oi.getResetGyroButton().onTrue(Commands.runOnce(WLDrive::zeroGyroscope, WLDrive));
  }
}
