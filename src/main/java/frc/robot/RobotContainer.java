// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathPlannerCommands;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIOSpark;
import frc.robot.subsystems.algae.AlgaeIOVirtualSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.climber.ClimberIOVirtualSim;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.arm.ArmIO;
import frc.robot.subsystems.coral.arm.ArmIOMMSpark;
import frc.robot.subsystems.coral.arm.ArmIOVirtualSim;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.elevator.ElevatorIO;
import frc.robot.subsystems.coral.elevator.ElevatorIOSpark;
import frc.robot.subsystems.coral.elevator.ElevatorIOVirtualSim;
import frc.robot.subsystems.coral.intake.Intake;
import frc.robot.subsystems.coral.intake.IntakeIO;
import frc.robot.subsystems.coral.intake.IntakeIOSpark;
import frc.robot.subsystems.coral.intake.IntakeIOVirtualSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.OdometryHealthMonitor;
import frc.robot.util.RobotStatus;
import frc.robot.util.Visualizer;
import java.io.IOException;
import java.util.Comparator;
import java.util.stream.Stream;
import lombok.Getter;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  @Getter private final Vision vision;
  @Getter private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  @Getter private Climber climber;
  @Getter private final CoralSystem coralSystem;
  @Getter private final Algae algae;
  @Getter private Visualizer visualizer;
  private final IndicatorLight indicatorLight;
  private OperatorInterface oi = new OperatorInterface() {};
  private LoggedDashboardChooser<Command> autoChooser;

  @Getter private final OdometryHealthMonitor odometryHealthMonitor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        intake = new Intake(new IntakeIOSpark() {});
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::addVisionMeasurementForLogging,
                new VisionIOPhotonVision(frontRightCam, robotToFrontRightCam),
                new VisionIOPhotonVision(backRightCam, robotToBackRightCam),
                new VisionIOPhotonVision(elevatorFrontCam, robotToElevatorFrontCam),
                new VisionIOPhotonVision(elevatorBackCam, robotToElevatorBackCam));

        elevator = new Elevator(new ElevatorIOSpark() {});
        algae = new Algae(new AlgaeIOSpark());
        climber = new Climber(new ClimberIOSpark() {});
        arm = new Arm(new ArmIOMMSpark() {});
        coralSystem = new CoralSystem(elevator, arm, intake);
        indicatorLight = new IndicatorLight();
        indicatorLight.setupLightingSuppliers(
            coralSystem::getCurrentCoralPreset,
            coralSystem.coralSystemPresetChooser::getSelected,
            coralSystem::getTargetCoralPreset,
            coralSystem::isHaveCoral);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::addVisionMeasurementForLogging,
                new VisionIOPhotonVisionSim(frontRightCam, robotToFrontRightCam, drive::getPose),
                new VisionIOPhotonVisionSim(backRightCam, robotToBackRightCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    elevatorFrontCam, robotToElevatorFrontCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    elevatorBackCam, robotToElevatorBackCam, drive::getPose));
        elevator = new Elevator(new ElevatorIOVirtualSim() {});
        arm = new Arm(new ArmIOVirtualSim() {});
        intake = new Intake(new IntakeIOVirtualSim() {});
        coralSystem = new CoralSystem(elevator, arm, intake);
        algae = new Algae(new AlgaeIOVirtualSim());
        climber = new Climber(new ClimberIOVirtualSim() {});
        indicatorLight = new IndicatorLight();
        indicatorLight.setupLightingSuppliers(
            coralSystem::getCurrentCoralPreset,
            coralSystem.coralSystemPresetChooser::getSelected,
            coralSystem::getTargetCoralPreset,
            coralSystem::isHaveCoral);

        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::addVisionMeasurementForLogging,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeIO() {});
        coralSystem = new CoralSystem(elevator, arm, intake);
        algae = new Algae(new AlgaeIO() {});
        climber = new Climber(new ClimberIO() {});
        indicatorLight = new IndicatorLight();
        indicatorLight.setupLightingSuppliers(
            coralSystem::getCurrentCoralPreset,
            coralSystem.coralSystemPresetChooser::getSelected,
            coralSystem::getTargetCoralPreset,
            coralSystem::isHaveCoral);
        break;
    }

    visualizer =
        new Visualizer(
            drive::getPose,
            elevator::getHeightInMeters,
            arm::getCurrentAngleDEG,
            coralSystem::isHaveCoral,
            algae::haveAlgae,
            algae::getDeployPositionAngle);

    odometryHealthMonitor = new OdometryHealthMonitor(drive, vision);

    if (elevator != null) {
      elevator.setTargetPreset(CoralSystemPresets.STARTUP);
    }

    if (arm != null) {
      arm.setTargetPreset(CoralSystemPresets.STARTUP);
    }

    // give static access to certain methods across subsystems
    RobotStatus.initialize(drive, coralSystem, vision, climber, algae);

    PathPlannerCommands.Setup(coralSystem, drive, vision);
    SetupAutoChooser();
    updateOI();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }
    normalModeOI();
  }

  public void normalModeOI() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();
    ButtonsAndDashboardBindings.configureBindings(
        oi, drive, coralSystem, climber, algae, vision, indicatorLight);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Pose2d getStartPose() {
    try {
      Pose2d startingPose =
          PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.get().getName())
              .get(0)
              .getStartingHolonomicPose()
              .get();
      return startingPose;
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      // Handle the IOException appropriately
    }
    return null;
  }

  public void SetupAutoChooser() {

    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Chooser",
            AutoBuilder.buildAutoChooserWithOptionsModifier(
                "compDefaultMoveOnly",
                stream -> {
                  Stream<PathPlannerAuto> modified =
                      Constants.isCompetition
                          ? stream.filter(auto -> auto.getName().startsWith("comp"))
                          : stream;
                  return modified.sorted(Comparator.comparing(PathPlannerAuto::getName));
                }));

    // add these if we aren't at competition
    if (!Constants.isCompetition) {
      // Set up SysId routines
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  public void teleopInitTurnSmartDriveOn() {
    DriveCommands.setSmartDriveCmd(
            drive,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            coralSystem::isHaveCoral,
            climber::isClimberDeployed,
            coralSystem.getElevator()::getHeightInInches)
        .schedule();
  }

  public void autoInitTurnSmartDriveOff() {
    DriveCommands.setNormalDriveCmd(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .schedule();
  }
}
