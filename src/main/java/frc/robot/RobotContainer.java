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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CoralSystemCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOMMSpark;
import frc.robot.subsystems.arm.ArmIOVirtualSim;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.elevator.ElevatorIOVirtualSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.IntakeIOVirtualSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Visualizer;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;
  @Getter private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final CoralSystem coralSystem;
  @Getter private Visualizer visualizer;
  private final IndicatorLight indicatorLight;
  private OperatorInterface oi = new OperatorInterface() {};
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
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
                new VisionIOPhotonVision(frontRightCam, robotToFrontRightCam),
                new VisionIOPhotonVision(backRightCam, robotToBackRightCam));
        // new VisionIOPhotonVision(elevatorFrontCam, robotToElevatorFrontCam),
        // new VisionIOPhotonVision(elevatorBackCam, robotToElevatorBackCam));

        elevator = new Elevator(new ElevatorIOSpark() {});
        arm = new Arm(new ArmIOMMSpark() {});
        intake = new Intake(new IntakeIOSpark() {});
        coralSystem = new CoralSystem(elevator, arm, intake);
        indicatorLight = new IndicatorLight();
        indicatorLight.setupLightingSuppliers(
            coralSystem::getCurrentCoralPreset,
            coralSystem.coralSystemPresetChooser::getSelected,
            coralSystem::getTargetCoralPreset,
            coralSystem::isCoralInRobot);
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
        indicatorLight = new IndicatorLight();
        indicatorLight.setupLightingSuppliers(
            coralSystem::getCurrentCoralPreset,
            coralSystem.coralSystemPresetChooser::getSelected,
            coralSystem::getTargetCoralPreset,
            coralSystem::isCoralInRobot);

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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = null;
        arm = null;
        coralSystem = null;
        intake = null;
        indicatorLight = null;
        break;
    }

    visualizer =
        new Visualizer(
            drive::getPose,
            elevator::getHeightInMeters,
            arm::getAngleDEG,
            coralSystem::isCoralInRobot);

    SetupSmartDashboardUI();
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
    WLButtons.configureButtonBindings(oi, drive, coralSystem, indicatorLight);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void SetupSmartDashboardUI() {
    if (elevator != null) {
      elevator.setPosition(0.0);
      SmartDashboard.putNumber("Elevator Goal (in)", 0.0);
      SmartDashboard.putData(
          "Set Elevator", CoralSystemCommands.setElevatorPositionFromDashboard(coralSystem));
    }

    if (arm != null) {
      SmartDashboard.putNumber("Arm Goal (DEG)", 90.0);
      SmartDashboard.putData(
          "Set Arm", CoralSystemCommands.setArmPositionFromDashboard(coralSystem));
    }

    SmartDashboard.putData(
        "DriveToClosestLEFTPole",
        DriveToCommands.driveToPole(
            drive, // The Drive subsystem
            true, // isLeftPole = true
            () -> 0.0, // Default X joystick input (stationary for dashboard testing)
            () -> 0.0, // Default Y joystick input
            () -> 0.0, // Default rotation joystick input
            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    SmartDashboard.putData(
        "DriveToClosestRIGHTPole",
        DriveToCommands.driveToPole(
            drive, // The Drive subsystem
            false, // isLeftPole = false
            () -> 0.0, // Default X joystick input (stationary for dashboard testing)
            () -> 0.0, // Default Y joystick input
            () -> 0.0, // Default rotation joystick input
            FieldConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    SmartDashboard.putData(
        "Toggle Vision", Commands.runOnce(drive::toggleVision).ignoringDisable(true));
  }

  public void SetupAutoChooser() {
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
