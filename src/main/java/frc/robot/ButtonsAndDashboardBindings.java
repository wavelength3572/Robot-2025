package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static CoralSystem coralSystem;
  private static IndicatorLight indicatorLight;
  private static Climber climber;

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
      Climber climber,
      IndicatorLight indicatorLight) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.coralSystem = coralSystem;
    ButtonsAndDashboardBindings.indicatorLight = indicatorLight;
    ButtonsAndDashboardBindings.climber = climber;

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
            coralSystem::isCoralInRobot,
            climber::isClimberDeployed,
            coralSystem.getElevator()::getHeightInInches));

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

    SmartDashboard.putData("Algae Alignment", AlgaeCommands.AlgaeAlignment(drive, coralSystem, oi));
    SmartDashboard.putData(
        "Algae Dislodge", AlgaeCommands.createDislodgeSequence(drive, coralSystem, oi));

    SmartDashboard.putData("Deploy Climber", Commands.runOnce(climber::deployClimber));
    SmartDashboard.putData("Stow Climber", Commands.runOnce(climber::stowClimber));
    SmartDashboard.putData(
        "Toggle Cage Alignment",
        Commands.runOnce(drive.getStrategyManager()::toggleAutoCageAlignmentMode));
  }

  private static void configureDriverButtonBindings() {
    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

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

    oi.getRightJoyLeftButton()
        .onTrue(
            DriveToCommands.driveToPole(
                drive,
                true,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
                coralSystem::isCoralInRobot));

    oi.getRightJoyRightButton()
        .onTrue(
            DriveToCommands.driveToPole(
                drive,
                false,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
                coralSystem::isCoralInRobot));
  }
}
