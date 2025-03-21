package frc.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants.CageTarget;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.AlignAndScore;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.commands.ScoreCoralInTeleopCommand;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.algae.Algae;
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

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDashboardBindings();
  }

  /****************************** */
  /*** DASHBOARD BINDINGS ****** */
  /****************************** */

  private static void configureDashboardBindings() {

    // Dashboard Buttons to Mimic the Button Box
    SmartDashboard.putData(
        "Shelf - L1",
        Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L1_SCORE)));
    SmartDashboard.putData(
        "Low - L2",
        Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L2))); // L2
    SmartDashboard.putData(
        "Mid - L3",
        Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L3))); // L3
    SmartDashboard.putData(
        "High - L4",
        Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L4))); // L4

    SmartDashboard.putData("Pickup Coral", createPickupCoralCommand());
    SmartDashboard.putData("Reef Action", createReefActionCommand()); // Score OR Dislodge
    SmartDashboard.putData("Prep L1 Dislodge", createPrepL1DislodgeCommand());
    SmartDashboard.putData("Prep L2 Dislodge", createPrepL2DislodgeCommand());

    SmartDashboard.putData("Recover E&A", Commands.runOnce(coralSystem::recoverArmAndElevator));

    SmartDashboard.putData(
        "Deploy & Capture Algae",
        Commands.runOnce(algae::pullAlgae)); // Deploy Algae Collector & Capture Algae

    // This is one button on the Button Box, not sure how to simulate the same thing
    // from dashbaord
    SmartDashboard.putData(
        "Deploy & Process Algae",
        Commands.runOnce(algae::pushAlgae)); // Depoloy Algae Collector & Process Algae

    SmartDashboard.putData(
        "Stow Algae Collector", Commands.runOnce(algae::stowAlgae)); // Stow Algae Collector

    // Dashboard Toggles
    SmartDashboard.putData(
        "Toggle Vision", Commands.runOnce(vision::toggleVision).ignoringDisable(true));

    SmartDashboard.putData(
        "Toggle Smart Drive",
        DriveCommands.toggleSmartDriveCmd(
                drive,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                coralSystem::isHaveCoral,
                climber::isClimberDeployed,
                coralSystem.getElevator()::getHeightInInches)
            .ignoringDisable(true));

    SmartDashboard.putData(
        "Toggle Cage Alignment Mode",
        Commands.runOnce(drive.getStrategyManager()::toggleAutoCageAlignmentMode));

    SmartDashboard.putData(
        "Toggle Alignment Indicator",
        Commands.runOnce(indicatorLight::toggleBranchAlignmentIndicator).ignoringDisable(true));

    SmartDashboard.putData(
        "Toggle Elevator Limits Speed", Commands.runOnce(drive::toggleElevatorHeightLimitsSpeed));

    // SmartDashboard Alignment Buttons
    SmartDashboard.putData(
        "Closest A Pole",
        DriveToCommands.driveToPole(
            drive,
            true,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
            coralSystem::isHaveCoral));

    SmartDashboard.putData(
        "Closest B Pole",
        DriveToCommands.driveToPole(
            drive,
            false,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
            coralSystem::isHaveCoral));

    SmartDashboard.putData(
        "Closest Staged Algae", AlgaeCommands.AlgaeAlignment(drive, coralSystem, oi));

    SmartDashboard.putData(
        "Shorten Foot", Commands.runOnce(() -> climber.setRelayState(Relay.Value.kForward)));
  }

  /****************************** */
  /*** DRIVER BINDINGS ****** */
  /****************************** */

  private static void configureDriverButtonBindings() {
    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    setInitialCageSelectionToMatchSwitch();

    oi.getButtonDPosition0()
        .onTrue(
            Commands.runOnce(AlignmentUtils::setLeftCage)
                .ignoringDisable(true)
                .alongWith(Commands.runOnce(() -> climber.setSelectedCageTarget(CageTarget.LEFT)))
                .ignoringDisable(true));
    oi.getButtonDPosition1()
        .onTrue(
            Commands.runOnce(AlignmentUtils::setMidCage)
                .ignoringDisable(true)
                .alongWith(Commands.runOnce(() -> climber.setSelectedCageTarget(CageTarget.MID)))
                .ignoringDisable(true));
    oi.getButtonDPosition2()
        .onTrue(
            Commands.runOnce(AlignmentUtils::setRightCage)
                .ignoringDisable(true)
                .alongWith(Commands.runOnce(() -> climber.setSelectedCageTarget(CageTarget.RIGHT)))
                .ignoringDisable(true));

    oi.getRightJoyLeftButton().onTrue(AlignAndScore.create(drive, coralSystem, true));

    // .onTrue(
    //     DriveToCommands.driveToPole(
    //         drive,
    //         true,
    //         oi::getTranslateX,
    //         oi::getTranslateY,
    //         oi::getRotate,
    //         FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
    //         coralSystem::isHaveCoral));

    oi.getRightJoyRightButton()
        // .onTrue(
        //     DriveToCommands.driveToPole(
        //         drive,
        //         false,
        //         oi::getTranslateX,
        //         oi::getTranslateY,
        //         oi::getRotate,
        //         FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE,
        //         coralSystem::isHaveCoral));

        .onTrue(AlignAndScore.create(drive, coralSystem, false));

    oi.getRightJoyDownButton().onTrue(AlgaeCommands.AlgaeAlignment(drive, coralSystem, oi));

    oi.getRightJoyUpButton().onTrue(AlgaeCommands.AlgaeAlignment(drive, coralSystem, oi));

    oi.getButtonI()
        .onTrue(
            DriveCommands.toggleSmartDriveCmd(
                drive,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                coralSystem::isHaveCoral,
                climber::isClimberDeployed,
                coralSystem.getElevator()::getHeightInInches));
  }

  private static void setInitialCageSelectionToMatchSwitch() {
    if (oi.getButtonDPosition0().getAsBoolean()) {
      AlignmentUtils.setLeftCage();
      climber.setSelectedCageTarget(CageTarget.LEFT);
    } else if (oi.getButtonDPosition1().getAsBoolean()) {
      AlignmentUtils.setMidCage();
      climber.setSelectedCageTarget(CageTarget.MID);
    } else {
      AlignmentUtils.setRightCage();
      climber.setSelectedCageTarget(CageTarget.RIGHT);
    }
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {

    oi.getButtonBox1YAxisPositive()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (coralSystem.haveCoral)
                    coralSystem.setTargetPreset(CoralSystemPresets.L1_SCORE);
                  else coralSystem.setTargetPreset(CoralSystemPresets.L1_STOW);
                })); // L1

    oi.getButtonBox1YAxisNegative()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L2))); // L2
    oi.getButtonBox1XAxisNegative()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L3))); // L3
    oi.getButtonBox1XAxisPositive()
        .onTrue(Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L4))); // L4

    oi.getButtonBox1Button1() // Prep L1 Algae Dislodge
        .onTrue(createPrepL1DislodgeCommand());

    oi.getButtonBox1Button2() // Prep L2 Algae Dislodge
        .onTrue(createPrepL2DislodgeCommand());

    oi.getButtonBox1Button3() // Reef Action - Dislodge Algae Sequence OR score coral
        .onTrue(createReefActionCommand());

    oi.getButtonBox1Button4() // Pickup Coral
        .onTrue(createPickupCoralCommand());

    oi.getButtonBox1Button5() // Process Algae / Stow Collector
        .onTrue(Commands.runOnce(algae::pushAlgae))
        .onFalse(Commands.runOnce(algae::stowAlgae));

    oi.getButtonBox1Button6() // Deploy Collector & Capture Algae
        .onTrue(Commands.runOnce(algae::pullAlgae));
    // if we dont have a coral and PICKUP, dont move arm
    // otherwise move to L1_STOW

    oi.getButtonBox1Button7() // Deploy Climber - the big switch
        .onTrue(Commands.runOnce(climber::deployClimber))
        .onTrue(Commands.runOnce(coralSystem::deployClimberTriggered))
        .onTrue(Commands.runOnce(algae::deployClimberTriggered));

    oi.getButtonBox1Button8().onTrue(Commands.runOnce(climber::climb)); // Climb
  }

  // If you click the pickup button and you have coral then do nothing
  private static Command createPickupCoralCommand() {
    return Commands.runOnce(
        () -> {
          if (RobotStatus.haveCoral() == false) {
            coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
            coralSystem.getIntake().pullCoral();
          }
        });
  }

  private static Command createReefActionCommand() {
    return Commands.runOnce(
        () -> {
          if (coralSystem.currentCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
              || coralSystem.currentCoralPreset
                  == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2) {
            AlgaeCommands.createDislodgeSequence(drive, coralSystem, oi).schedule();
          } else {
            new ScoreCoralInTeleopCommand(coralSystem.getIntake()).schedule();
          }
        });
  }

  private static Command createPrepL2DislodgeCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2),
            coralSystem),
        new WaitUntilCommand(coralSystem::isAtGoal),
        new InstantCommand(
            () -> coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2),
            coralSystem));
  }

  private static Command createPrepL1DislodgeCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1),
            coralSystem),
        new WaitUntilCommand(coralSystem::isAtGoal),
        new InstantCommand(
            () -> coralSystem.setTargetPreset(CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1),
            coralSystem));
  }
}
