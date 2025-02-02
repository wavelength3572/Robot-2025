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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CommandConstants;
import frc.robot.commands.CoralSystemCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCommands;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.arm.ArmConstants;
import frc.robot.subsystems.coral.arm.ArmIOSim;
import frc.robot.subsystems.coral.arm.ArmIOSpark;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.elevator.ElevatorConstants;
import frc.robot.subsystems.coral.elevator.ElevatorIOPPCSim;
import frc.robot.subsystems.coral.elevator.ElevatorIOSpark;
import frc.robot.subsystems.coral.endeffector.EndEffector;
import frc.robot.subsystems.coral.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.coral.endeffector.EndEffectorIOSpark;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
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
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endeffector;
  private final CoralSubsystem coralSubsystem; // Contains elevator, arm, endeffector

  private OperatorInterface oi = new OperatorInterface() {};

  private LoggedMechanism2d scoringSystem = new LoggedMechanism2d(.8382, 2.0);
  private LoggedMechanismRoot2d root = scoringSystem.getRoot("Base", 0.51, 0.0);
  private LoggedMechanismLigament2d m_elevator =
      root.append(
          new LoggedMechanismLigament2d(
              "Elevator", ElevatorConstants.kGroundToElevator, 90, 2, new Color8Bit(Color.kBlue)));

  // New arm visualization:
  // For example, assume the arm has a fixed length defined in your constants:
  private LoggedMechanismLigament2d m_arm =
      m_elevator.append(
          new LoggedMechanismLigament2d(
              "Arm",
              ArmConstants.kArmLengthMeters, // The physical length of your arm.
              30, // Initial angle (in degrees).
              2, // Thickness (example value)
              new Color8Bit(Color.kRed)));

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new VisionIOPhotonVision(backRightCam, robotToBackRightCam));

        elevator = new Elevator(new ElevatorIOSpark());
        arm = new Arm(new ArmIOSpark());
        endeffector = new EndEffector(new EndEffectorIOSpark());
        coralSubsystem = new CoralSubsystem(elevator, arm, endeffector);
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

        elevator = new Elevator(new ElevatorIOPPCSim());
        arm = new Arm(new ArmIOSim());
        endeffector = new EndEffector(new EndEffectorIOSim());
        coralSubsystem = new CoralSubsystem(elevator, arm, endeffector);
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
        endeffector = null;
        coralSubsystem = null;

        break;
    }

    if (elevator != null) {
      elevator.setPosition(0.0);
      SmartDashboard.putNumber("Elevator Goal", 0.0);
      SmartDashboard.putData("Elevator", CoralSystemCommands.setElevatorPositionFromDashboard(coralSubsystem));
    }

    if (arm != null) {
      arm.setAngleDegrees(0.0);
      SmartDashboard.putNumber("Arm Goal", 0.0);
      SmartDashboard.putData("Arm", CoralSystemCommands.setArmAngleFromDashboard(coralSubsystem));
    }


    SmartDashboard.putData(
        "DriveToClosestLEFTPole",
        DriveToCommands.driveToPole(
            drive, // The Drive subsystem
            true, // isLeftPole = true
            () -> 0.0, // Default X joystick input (stationary for dashboard testing)
            () -> 0.0, // Default Y joystick input
            () -> 0.0, // Default rotation joystick input
            CommandConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));

    SmartDashboard.putData(
        "DriveToClosestRIGHTPole",
        DriveToCommands.driveToPole(
            drive, // The Drive subsystem
            false, // isLeftPole = false
            () -> 0.0, // Default X joystick input (stationary for dashboard testing)
            () -> 0.0, // Default Y joystick input
            () -> 0.0, // Default rotation joystick input
            CommandConstants.THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // autoChooser = new LoggedDashboardChooser<>("Auto");

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

    WLButtons.configureButtonBindings(oi, drive, coralSubsystem);

    // Configure some robot defaults based on current state of Controller Switches.
    // if (oi.getFieldRelativeButton().getAsBoolean()) {
    // System.out.println("Field Relative Button False");
    // drive.disableFieldRelative();
    // } else {
    // System.out.println("Field Relative Button True");
    // drive.enableFieldRelative();
    // }

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public LoggedMechanism2d getElevator() {
    // Update the Elevator 2D Mech
    m_elevator.setLength(ElevatorConstants.kGroundToElevator + elevator.getHeightInMeters());
    return scoringSystem;
  }

  public LoggedMechanism2d getArm() {
    // Update the Arm 2D Mech
    m_arm.setAngle(arm.getAngleInDegrees());
    return scoringSystem;
  }
}
