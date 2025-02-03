package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.elevator.Elevator;
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

    // Create the 2D mechanism visualizer using constants.
    visualizer =
        new LoggedMechanism2d(
            CoralSubsystemConstants.VISUALIZER_WIDTH, CoralSubsystemConstants.VISUALIZER_HEIGHT);

    // Create the root for the visualization.
    root =
        visualizer.getRoot(
            CoralSubsystemConstants.ROOT_NAME,
            CoralSubsystemConstants.ROOT_X_POSITION,
            CoralSubsystemConstants.ROOT_Y_POSITION);

    // Create the elevator ligament. The length is based on an initial constant plus the elevator’s
    // current height.
    elevatorLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                CoralSubsystemConstants.ELEVATOR_INITIAL_LENGTH + elevator.getHeightInMeters(),
                CoralSubsystemConstants.ELEVATOR_ANGLE_DEGREES,
                CoralSubsystemConstants.ELEVATOR_THICKNESS,
                CoralSubsystemConstants.ELEVATOR_COLOR));

    // Create the arm ligament.
    armLigament =
        elevatorLigament.append(
            new LoggedMechanismLigament2d(
                "Arm",
                CoralSubsystemConstants.ARM_LENGTH,
                CoralSubsystemConstants.ARM_INITIAL_ANGLE_DEGREES,
                CoralSubsystemConstants.ARM_THICKNESS,
                CoralSubsystemConstants.ARM_COLOR));

    // Create the end effector ligament.
    endEffectorLigament =
        armLigament.append(
            new LoggedMechanismLigament2d(
                "EndEffector",
                CoralSubsystemConstants.ENDEFFECTOR_LENGTH,
                CoralSubsystemConstants.ENDEFFECTOR_INITIAL_ANGLE_DEGREES,
                CoralSubsystemConstants.ENDEFFECTOR_THICKNESS,
                CoralSubsystemConstants.ENDEFFECTOR_COLOR));
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
    elevatorLigament.setLength(
        CoralSubsystemConstants.ELEVATOR_INITIAL_LENGTH + elevator.getHeightInMeters());

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
