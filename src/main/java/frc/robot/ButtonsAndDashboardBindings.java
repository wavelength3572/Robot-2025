package frc.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.Alignment.AlignAndScore.AlignAndScore;
import frc.robot.commands.Alignment.AlignAndScorePP.AlignAndScorePP;
import frc.robot.commands.Alignment.TwoStage.AlignAndScoreTwoStage;
import frc.robot.commands.ChooserAlignmentCommand;
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
import frc.robot.util.RobotStatus;
import java.util.function.Function;

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

    SmartDashboard.putData("Shelf - L1", createShelfCommand());
    SmartDashboard.putData("Low - L2", createPresetCommand(CoralSystemPresets.L2));
    SmartDashboard.putData("Mid - L3", createPresetCommand(CoralSystemPresets.L3));
    SmartDashboard.putData("High - L4", createPresetCommand(CoralSystemPresets.L4));

    SmartDashboard.putData("Pickup Coral", createPickupCoralCommand());
    SmartDashboard.putData("Reef Action", createReefActionCommand()); // Score OR Dislodge
    SmartDashboard.putData("Prep L1 Dislodge", createPrepL1DislodgeCommand());
    SmartDashboard.putData("Prep L2 Dislodge", createPrepL2DislodgeCommand());

    SmartDashboard.putData("Recover E&A", Commands.runOnce(coralSystem::recoverArmAndElevator));

    SmartDashboard.putData("Deploy & Capture Algae", getDeployAndCaptureAlgaeCommand());

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
                algae::haveAlgae,
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

    SmartDashboard.putData(
        "Toggle Staged Pre-scoring",
        Commands.runOnce(coralSystem::toggleStagedPrescoring)
            .ignoringDisable(true)); // This will toggle the staged algae mode for the coral system.

    // SmartDashboard Alignment Buttons
    SmartDashboard.putData(
        "Closest A Pole",
        DriveToCommands.driveToPole(
            drive, true, FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    SmartDashboard.putData(
        "Closest B Pole",
        DriveToCommands.driveToPole(
            drive, false, FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    SmartDashboard.putData(
        "Closest Staged Algae", AlgaeCommands.AlgaeAlignment(drive, coralSystem, oi));

    SmartDashboard.putData(
        "Shorten Foot", Commands.runOnce(() -> climber.setRelayState(Relay.Value.kForward)));

    SmartDashboard.putData(
        "Put Out Foot", Commands.runOnce(() -> climber.setRelayState(Relay.Value.kReverse)));

    SmartDashboard.putData(
        "Stop Foot", Commands.runOnce(() -> climber.setRelayState(Relay.Value.kOff)));
  }

  /****************************** */
  /*** DRIVER BINDINGS ****** */
  /****************************** */

  private static void configureDriverButtonBindings() {
    SetupAlignmentButtons();

    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    oi.getRightJoyDownButton().toggleOnTrue(AlgaeCommands.AlgaeAlignment(drive, coralSystem, oi));
    oi.getRightJoyUpButton().toggleOnTrue(AlgaeCommands.AlgaeAlignment(drive, coralSystem, oi));

    oi.getButtonI()
        .onTrue(
            DriveCommands.toggleSmartDriveCmd(
                drive,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                coralSystem::isHaveCoral,
                algae::haveAlgae,
                climber::isClimberDeployed,
                coralSystem.getElevator()::getHeightInInches));
  }

  private static void SetupAlignmentButtons() {
    SendableChooser<Function<Boolean, Command>> alignmentChooser = new SendableChooser<>();
    alignmentChooser.setDefaultOption(
        "Two Stage", (isLeftPole) -> AlignAndScoreTwoStage.create(drive, coralSystem, isLeftPole));
    alignmentChooser.addOption(
        "AlignAndScorePP", (isLeftPole) -> AlignAndScorePP.create(drive, coralSystem, isLeftPole));
    alignmentChooser.addOption(
        "AlignAndScore", (isLeftPole) -> AlignAndScore.create(drive, coralSystem, isLeftPole));

    SmartDashboard.putData("Alignment Strategy", alignmentChooser);
    ChooserAlignmentCommand alignLeftCommand = new ChooserAlignmentCommand(alignmentChooser, true);
    ChooserAlignmentCommand alignRightCommand =
        new ChooserAlignmentCommand(alignmentChooser, false);
    oi.getRightJoyLeftButton().toggleOnTrue(alignLeftCommand);
    oi.getRightJoyRightButton().toggleOnTrue(alignRightCommand);
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {

    oi.getButtonBox1YAxisPositive().onTrue(createShelfCommand()); // L1
    oi.getButtonBox1YAxisNegative().onTrue(createPresetCommand(CoralSystemPresets.L2)); // L2
    oi.getButtonBox1XAxisNegative().onTrue(createPresetCommand(CoralSystemPresets.L3)); // L3
    oi.getButtonBox1XAxisPositive().onTrue(createPresetCommand(CoralSystemPresets.L4)); // L4

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
        .onTrue(getDeployAndCaptureAlgaeCommand());

    oi.getButtonBox1Button7() // Deploy Climber - the big switch
        .onTrue(Commands.runOnce(climber::deployClimber))
        .onTrue(Commands.runOnce(coralSystem::deployClimberTriggered))
        .onTrue(Commands.runOnce(algae::deployClimberTriggered));

    oi.getButtonBox1Button8().onTrue(Commands.runOnce(climber::climb)); // Climb
  }

  private static Command createPickupCoralCommand() {
    return Commands.runOnce(
        () -> {
          if (RobotStatus.haveCoral() == false) {
            coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
            coralSystem.setQueuedFinalPreset(null);
            // coralSystem.getIntake().pullCoral();
          } else {
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

  public static Command getDeployAndCaptureAlgaeCommand() {
    return Commands.sequence(
        Commands.runOnce(algae::pullAlgae), // PULL algae
        new ConditionalCommand(
            Commands.none(), // If the arm is in pickup or L1_STOW, do nothing
            Commands.runOnce(() -> coralSystem.setTargetPreset(CoralSystemPresets.L1_STOW)),
            () ->
                (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.PICKUP
                    || (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L1_STOW))));
  }

  // L1 which checks for coral presence:
  private static Command createShelfCommand() {
    return Commands.runOnce(
        () -> {
          if (coralSystem.haveCoral) coralSystem.setTargetPreset(CoralSystemPresets.L1_SCORE);
          else coralSystem.setTargetPreset(CoralSystemPresets.L1_STOW);
        });
  }

  // For L2, L3, and L4 which use the staged pre-scoring logic:
  private static Command createPresetCommand(CoralSystemPresets preset) {
    return Commands.runOnce(
        () -> {
          if (coralSystem.isStagedPreScoringOn()) {
            // Check if we are in L1/L2/L3 and nothing is queued
            if (coralSystem.getQueuedFinalPreset() == null
                && (coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L2_FAR
                    || coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L3_FAR
                    || coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L4_FAR
                    || coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L2
                    || coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L3
                    || coralSystem.getCurrentCoralPreset() == CoralSystemPresets.L4)) {
              // Manual override: immediately set the new preset.
              coralSystem.setTargetPreset(preset);
              // Clear any queued preset.
              coralSystem.setQueuedFinalPreset(null);
            } else {
              // We are in staging mode. Now check if a preset is already queued.
              if (coralSystem.getQueuedFinalPreset() != null
                  && coralSystem.getQueuedFinalPreset().equals(preset)) {
                // Button pressed twice: commit to the preset.
                coralSystem.setTargetPreset(preset);
                coralSystem.setQueuedFinalPreset(null);
              } else {
                // First press: remain in staging and queue this preset.
                coralSystem.setTargetPreset(CoralSystemPresets.STAGED_FOR_SCORING);
                coralSystem.setQueuedFinalPreset(preset);
              }
            }
          } else {
            // Staged pre-scoring is off: go directly to the preset.
            coralSystem.setTargetPreset(preset);
          }
        });
  }
}
