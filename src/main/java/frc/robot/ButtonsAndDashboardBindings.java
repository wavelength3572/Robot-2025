package frc.robot;

import static frc.robot.subsystems.coral.CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1;
import static frc.robot.subsystems.coral.CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.commands.NamedCommands.ScoreCoralCommand;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AlignmentUtils;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static CoralSystem coralSystem;
  private static IndicatorLight indicatorLight;
  private static Climber climber;
  private static Algae algae;
  private static Vision vision;

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
      Algae algae,
      Vision vision,
      IndicatorLight indicatorLight) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.coralSystem = coralSystem;
    ButtonsAndDashboardBindings.indicatorLight = indicatorLight;
    ButtonsAndDashboardBindings.climber = climber;
    ButtonsAndDashboardBindings.algae = algae;
    ButtonsAndDashboardBindings.vision = vision;

    drive.setDriveModeNormal();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    configureDriverButtonBindings();
    configureOperatorButtonBindings();

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
        "Toggle Vision", Commands.runOnce(vision::toggleVision).ignoringDisable(true));

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

    // Algae Deploy Angle Control (Adjust arm angle)
    SmartDashboard.putNumber("Set Algae Rotation Target", 0.0); // Default to 0 degrees
    SmartDashboard.putData(
        "Apply Algae Rotation Target",
        Commands.runOnce(
            () -> {
              double angle =
                  SmartDashboard.getNumber(
                      "Set Algae Angle Target", AlgaeConstants.kAlgaeDeployInitalAngle);
              algae.setDeployPositionAngle(angle);
            }));

    SmartDashboard.putData("Stow Algae", Commands.runOnce(algae::stowAlgae));
    SmartDashboard.putData("Deploy Algae", Commands.runOnce(algae::deployAlgae));

    // Algae Motor Speed Control (Adjust intake/outtake speed)
    SmartDashboard.putNumber("Set Algae Speed", 0.0); // Default to 0 speed
    SmartDashboard.putData(
        "Apply Algae Speed",
        Commands.runOnce(
            () -> {
              double speed = SmartDashboard.getNumber("Set Algae Speed", 0.0);
              algae.setSpeed(speed);
            }));
  }

  private static void configureDriverButtonBindings() {
    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    oi.getButtonV()
        .onTrue(Commands.runOnce(() -> coralSystem.setCoralInRobot(true), coralSystem))
        .onFalse(Commands.runOnce(() -> coralSystem.setCoralInRobot(false), coralSystem));

    // oi.getButtonFPosition0() // Push Intake
    // .onTrue(
    // Commands.runOnce(
    // () -> {
    // coralSystem.getIntake().pushCoral();
    // }))
    // .onFalse(
    // Commands.runOnce(
    // () -> {
    // coralSystem.getIntake().stopIntake();
    // ;
    // }));
    // oi.getButtonFPosition2() // Pull Intake
    // .onTrue(
    // Commands.runOnce(
    // () -> {
    // coralSystem.getIntake().pullCoral();
    // }))
    // .onFalse(
    // Commands.runOnce(
    // () -> {
    // coralSystem.getIntake().stopIntake();
    // ;
    // }));

    // oi.getButtonFPosition0() // Push Intake
    // .onTrue(
    // Commands.runOnce(
    // () -> {
    // algae.pushAlgae();
    // }))
    // .onFalse(
    // Commands.runOnce(
    // () -> {
    // algae.stopAlgae();
    // }));
    // oi.getButtonFPosition2() // Pull Intake
    // .onTrue(
    // Commands.runOnce(
    // () -> {
    // algae.pullAlgae();
    // }))
    // .onFalse(
    // Commands.runOnce(
    // () -> {
    // algae.stopAlgae();
    // }));

    oi.getButtonFPosition0() // Push Algae Arm
        .onTrue(
            Commands.runOnce(
                () -> {
                  algae.deployAlgae();
                }));
    oi.getButtonFPosition2() // Pull Algae Arm
        .onTrue(
            Commands.runOnce(
                () -> {
                  algae.stowAlgae();
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

  private static void configureOperatorButtonBindings() {

    oi.getButtonBox1Button3()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (coralSystem.currentCoralPreset == PREPARE_DISLODGE_LEVEL_1
                      || coralSystem.currentCoralPreset == PREPARE_DISLODGE_LEVEL_2) {
                    AlgaeCommands.createDislodgeSequence(drive, coralSystem, oi).schedule();
                  } else {
                    new ScoreCoralCommand(coralSystem.getIntake()).schedule();
                  }
                }));

    // oi.getButtonBox1Button7().onTrue(Commands.runOnce(climber::deployClimber));
    // oi.getButtonBox1Button8().onTrue(Commands.runOnce(climber::stowClimber));

    oi.getButtonBox1Button6()
        .onTrue(
            Commands.runOnce( // this is the collect algae - YELLOW INTAKE BUTTON
                () -> {
                  algae.deployAlgae(); // deploy the algae mechanism
                  algae.pullAlgae();
                }));

    oi.getButtonBox1Button5()
        .onTrue(
            Commands.runOnce( // this is the process algae button
                () -> {
                  algae.pushAlgae(); // run algae intake
                }))
        .onFalse(
            Commands.runOnce( // this is the process algae button
                () -> {
                  algae.stowAlgae(); // does mechanism need to move?
                  algae.stopAlgae(); // run algae intake
                }));

    oi.getButtonBox1YAxisPositive()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L1))); // L1
    oi.getButtonBox1YAxisNegative()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L2))); // L2
    oi.getButtonBox1XAxisNegative()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L3))); // L3
    oi.getButtonBox1XAxisPositive()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L4))); // L4

    oi.getButtonBox1Button1()
        .onTrue(
            Commands.runOnce(
                () -> coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1)));
    oi.getButtonBox1Button2()
        .onTrue(
            Commands.runOnce(
                () -> coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2)));

    oi.getButtonBox1Button4()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
                  coralSystem.getIntake().pullCoral();
                }));
  }
}
