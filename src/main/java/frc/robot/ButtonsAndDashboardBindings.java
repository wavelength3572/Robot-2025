package frc.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
import frc.robot.util.RobotStatus;

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

  /****************************** */
  /*** DASHBOARD BINDINGS ****** */
  /****************************** */

  private static void configureDashboardBindings() {

    SmartDashboard.putData(
        "Set Coral In Robot",
        Commands.runOnce(() -> coralSystem.setCoralInRobot(true)).ignoringDisable(true));

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
        "Toggle Elevator Height Speed Limits",
        Commands.runOnce(drive::toggleElevatorHeightLimitsSpeed));

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

    SmartDashboard.putData(
        "Toggle Cage Alignment Mode",
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

    SmartDashboard.putData(
        "Deploy Algae Collector",
        Commands.runOnce( // this is the collect algae - YELLOW INTAKE BUTTON
            () -> {
              algae.deployAlgae(); // deploy the algae mechanism
              algae.pullAlgae();
            }));

    SmartDashboard.putData(
        "Process Algae",
        Commands.runOnce( // this is the process algae button
            () -> {
              algae.pushAlgae(); // run algae intake
            }));

    SmartDashboard.putData(
        "Stow Algae",
        Commands.runOnce( // this is the process algae button
            () -> {
              algae.stowAlgae(); // does mechanism need to move?
              algae.stopAlgae(); // run algae intake
            }));

    SmartDashboard.putData(
        "Prepare Dislodge L1",
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1)),
            new WaitUntilCommand(coralSystem::isAtGoal),
            new InstantCommand(
                () ->
                    coralSystem.setSimultaneousTargetPreset(
                        CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1))));

    SmartDashboard.putData(
        "Dislodge",
        Commands.runOnce(
            () -> {
              if (coralSystem.currentCoralPreset
                      == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                  || coralSystem.currentCoralPreset
                      == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2) {
                AlgaeCommands.createDislodgeSequence(drive, coralSystem, oi).schedule();
              } else {
                new ScoreCoralCommand(coralSystem.getIntake()).schedule();
              }
            }));

    SmartDashboard.putData(
        "Toggle Vision Alignment",
        Commands.runOnce(indicatorLight::toggleBranchAlignment).ignoringDisable(true));
  }

  /****************************** */
  /*** DRIVER BINDINGS ****** */
  /****************************** */

  private static void configureDriverButtonBindings() {
    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    oi.getButtonFPosition0() // Push Relay Arm
        .onTrue(
            Commands.runOnce(
                () -> {
                  climber.setRelayState(Relay.Value.kForward);
                }));

    oi.getButtonFPosition2() // Pull Relay Arm
        .onTrue(
            Commands.runOnce(
                () -> {
                  climber.setRelayState(Relay.Value.kReverse);
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

    // oi.getRightJoyLeftButton()
    // .onTrue(
    // DriveToCommands.driveToPole(
    // drive,
    // true,
    // oi::getTranslateX,
    // oi::getTranslateY,
    // oi::getRotate,
    // FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
    // coralSystem::isCoralInRobot));

    // oi.getRightJoyRightButton()
    // .onTrue(
    // DriveToCommands.driveToPole(
    // drive,
    // false,
    // oi::getTranslateX,
    // oi::getTranslateY,
    // oi::getRotate,
    // FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
    // coralSystem::isCoralInRobot));
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {

    oi.getButtonBox1Button3() // Reef Button - So either dislodge sequence or spit out coral
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (coralSystem.currentCoralPreset
                          == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                      || coralSystem.currentCoralPreset
                          == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2) {
                    AlgaeCommands.createDislodgeSequence(drive, coralSystem, oi).schedule();
                  } else {
                    new ScoreCoralCommand(coralSystem.getIntake()).schedule();
                  }
                }));

    oi.getButtonBox1Button7() // the big switch
        .onTrue(Commands.runOnce(climber::deployClimber))
        .onTrue(Commands.runOnce(coralSystem::deployClimberTriggered))
        .onTrue(Commands.runOnce(algae::deployClimberTriggered));

    oi.getButtonBox1Button8().onTrue(Commands.runOnce(climber::climb));

    // oi.getButtonBox1Button6()
    // .onTrue(
    // Commands.runOnce( // this is the collect algae - YELLOW INTAKE BUTTON
    // () -> {
    // algae.deployAlgae(); // deploy the algae mechanism
    // algae.pullAlgae();
    // }));

    // oi.getButtonBox1Button5()
    // .onTrue(
    // Commands.runOnce( // this is the process algae button
    // () -> {
    // algae.pushAlgae(); // run algae intake
    // }))
    // .onFalse(
    // Commands.runOnce( // this is the process algae button
    // () -> {
    // algae.stowAlgae(); // does mechanism need to move?
    // algae.stopAlgae(); // run algae intake
    // }));

    oi.getButtonBox1YAxisPositive()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L1))); // L1
    oi.getButtonBox1YAxisNegative()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L2))); // L2
    oi.getButtonBox1XAxisNegative()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L3))); // L3
    oi.getButtonBox1XAxisPositive()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L4))); // L4

    oi.getButtonBox1Button1() // Prepare to dislodge L1 Algae
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        coralSystem.setTargetPreset(
                            CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1),
                    coralSystem),
                new WaitUntilCommand(coralSystem::isAtGoal),
                new InstantCommand(
                    () ->
                        coralSystem.setSimultaneousTargetPreset(
                            CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1),
                    coralSystem)));

    oi.getButtonBox1Button2() // Prepare to dislodge L2 Algae
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        coralSystem.setTargetPreset(
                            CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2),
                    coralSystem),
                new WaitUntilCommand(coralSystem::isAtGoal),
                new InstantCommand(
                    () ->
                        coralSystem.setSimultaneousTargetPreset(
                            CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2),
                    coralSystem)));

    oi.getButtonBox1Button4() // Pickup button
        // If you click the pickup button and you have coral then
        // make the button act like you pressed the reef score button.
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotStatus.haveCoral() == false) {
                    coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
                    coralSystem.getIntake().pullCoral();
                  }
                }));
  }
}
