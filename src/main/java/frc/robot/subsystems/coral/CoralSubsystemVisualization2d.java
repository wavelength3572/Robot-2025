package frc.robot.subsystems.coral;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.elevator.ElevatorConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class CoralSubsystemVisualization2d {
  // Toggle for enabling/disabling the 2D visualization.
  public static final boolean ENABLE_2D_VISUALIZATION =
      false; // Set to false to disable the 2D visualization

  private LoggedMechanism2d visualizer;
  private LoggedMechanismRoot2d root;
  private LoggedMechanismLigament2d elevatorLigament;
  private LoggedMechanismLigament2d armLigament;
  private LoggedMechanismLigament2d endEffectorLigament;

  // 2D Visualizer dimensions
  public static final double VISUALIZER_WIDTH = 0.8382;
  public static final double VISUALIZER_HEIGHT = 2.0;

  // 2D Root configuration
  public static final String ROOT_NAME = "Base";
  public static final double ROOT_X_POSITION = VISUALIZER_WIDTH / 2.0;
  public static final double ROOT_Y_POSITION = 0.0;

  // 2D Elevator visualization parameters
  // Replace the ElevatorConstants value with an appropriate literal (for example, 0.5 meters)
  public static final double ELEVATOR_INITIAL_LENGTH = ElevatorConstants.kGroundToElevator;
  public static final double ELEVATOR_ANGLE_DEGREES = 90.0;
  public static final double ELEVATOR_THICKNESS = 2.0;
  public static final Color8Bit ELEVATOR_COLOR = new Color8Bit(Color.kBlue);

  // 2D Arm visualization parameters
  // Replace the ArmConstants value with an appropriate literal (for example, 1.0 meters)
  //   public static final double ARM_LENGTH = ArmConstants.kArmLengthMeters;
  public static final double ARM_LENGTH = Units.inchesToMeters(12.25);

  public static final double ARM_INITIAL_ANGLE_DEGREES = 0;
  public static final double ARM_THICKNESS = 3.0;
  public static final Color8Bit ARM_COLOR = new Color8Bit(Color.kRed);

  // 2D EndEffector visualization parameters
  // Replace the EndEffectorConstants value with an appropriate literal (for example, 0.3 meters)
  public static final double ENDEFFECTOR_LENGTH = 0.2;
  public static final double ENDEFFECTOR_INITIAL_ANGLE_DEGREES = -90.0;
  public static final double ENDEFFECTOR_THICKNESS = 5.0;
  public static final Color8Bit ENDEFFECTOR_COLOR = new Color8Bit(Color.kBlanchedAlmond);

  /**
   * Constructs the 2D visualization for the Coral subsystem.
   *
   * @param elevator The elevator component (needed for initial offset).
   * @param arm The arm component (needed for initial angle).
   */
  public CoralSubsystemVisualization2d(Elevator elevator, Arm arm) {
    if (!ENABLE_2D_VISUALIZATION) {
      return;
    }

    // Create the 2D mechanism visualizer using local constants.
    visualizer = new LoggedMechanism2d(VISUALIZER_WIDTH, VISUALIZER_HEIGHT);

    // Create the root for the visualization.
    root = visualizer.getRoot(ROOT_NAME, ROOT_X_POSITION, ROOT_Y_POSITION);

    // Create the elevator ligament.
    // The length is based on an initial constant plus the elevator’s current height.
    elevatorLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                ELEVATOR_INITIAL_LENGTH + elevator.getHeightInMeters(),
                ELEVATOR_ANGLE_DEGREES,
                ELEVATOR_THICKNESS,
                ELEVATOR_COLOR));

    // Create the arm ligament.
    armLigament =
        elevatorLigament.append(
            new LoggedMechanismLigament2d(
                "Arm", ARM_LENGTH, ARM_INITIAL_ANGLE_DEGREES, ARM_THICKNESS, ARM_COLOR));

    // Create the end effector ligament.
    endEffectorLigament =
        armLigament.append(
            new LoggedMechanismLigament2d(
                "EndEffector",
                ENDEFFECTOR_LENGTH,
                ENDEFFECTOR_INITIAL_ANGLE_DEGREES,
                ENDEFFECTOR_THICKNESS,
                ENDEFFECTOR_COLOR));
  }

  /**
   * Updates the visualization to reflect current sensor readings.
   *
   * @param elevator The elevator component.
   * @param arm The arm component.
   */
  public void update(Elevator elevator, Arm arm) {
    if (!ENABLE_2D_VISUALIZATION) {
      return;
    }

    // Update the elevator ligament length: base length plus current height.
    elevatorLigament.setLength(ELEVATOR_INITIAL_LENGTH + elevator.getHeightInMeters());

    // Update the arm ligament angle based on the arm's calibrated angle.
    armLigament.setAngle(arm.getCalibratedAngleDegrees());

    // If you decide to make the end effector's visualization dynamic later, update it here.
  }

  /**
   * Returns the LoggedMechanism2d for display.
   *
   * @return the 2D visualizer, or null if disabled.
   */
  public LoggedMechanism2d getVisualizer() {
    return visualizer;
  }
}
