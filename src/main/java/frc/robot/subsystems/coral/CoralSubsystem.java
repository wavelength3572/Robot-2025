package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.elevator.ElevatorConstants;
import frc.robot.subsystems.coral.endeffector.EndEffector;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class CoralSubsystem extends SubsystemBase {
  // Subcomponents that handle their own I/O and control logic
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;

  private boolean haveCoral = false;
  private boolean haveAlgae = false;

  private final LoggedMechanism2d visualizer;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d elevatorLigament;
  private final LoggedMechanismLigament2d armLigament;
  private final LoggedMechanismLigament2d endEffectorLigament;

  /**
   * Constructs a CoralSubsystem with the provided subcomponents.
   *
   * @param elevator The Elevator component.
   * @param arm The Arm component.
   * @param endEffector The EndEffector component.
   */
  public CoralSubsystem(Elevator elevator, Arm arm, EndEffector endEffector) {
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;

    // Create the 2D mechanism visualizer using constants.
    visualizer =
        new LoggedMechanism2d(
            CoralSubsystemConstants.VISUALIZER_WIDTH, CoralSubsystemConstants.VISUALIZER_HEIGHT);

    root =
        visualizer.getRoot(
            CoralSubsystemConstants.ROOT_NAME,
            CoralSubsystemConstants.ROOT_X_POSITION,
            CoralSubsystemConstants.ROOT_Y_POSITION);

    elevatorLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                CoralSubsystemConstants.ELEVATOR_INITIAL_LENGTH + elevator.getHeightInMeters(),
                CoralSubsystemConstants.ELEVATOR_ANGLE_DEGREES,
                CoralSubsystemConstants.ELEVATOR_THICKNESS,
                CoralSubsystemConstants.ELEVATOR_COLOR));

    armLigament =
        elevatorLigament.append(
            new LoggedMechanismLigament2d(
                "Arm",
                CoralSubsystemConstants.ARM_LENGTH,
                CoralSubsystemConstants.ARM_INITIAL_ANGLE_DEGREES,
                CoralSubsystemConstants.ARM_THICKNESS,
                CoralSubsystemConstants.ARM_COLOR));

    endEffectorLigament =
        armLigament.append(
            new LoggedMechanismLigament2d(
                "EndEffector",
                CoralSubsystemConstants.ENDEFFECTOR_LENGTH,
                CoralSubsystemConstants.ENDEFFECTOR_INITIAL_ANGLE_DEGREES,
                CoralSubsystemConstants.ENDEFFECTOR_THICKNESS,
                CoralSubsystemConstants.ENDEFFECTOR_COLOR));
  }

  /** The periodic method updates all subcomponents and the visualization. */
  @Override
  public void periodic() {
    // Update subcomponents.
    elevator.update();
    arm.update();
    endEffector.update();

    Logger.recordOutput("Alignment/haveCoral", haveCoral());

    // Refresh visualizer based on the current state.
    updateVisualizers();
  }

  /** Updates the visualizer to reflect the current positions of the subcomponents. */
  private void updateVisualizers() {
    // Update the elevator's ligament length:
    // base length plus the current height.
    elevatorLigament.setLength(ElevatorConstants.kGroundToElevator + elevator.getHeightInMeters());

    // Update the arm ligament angle based on the arm's calibrated angle.
    armLigament.setAngle(arm.getCalibratedAngleDegrees());

    // If the end effector angle is dynamic, update its angle here.
    // We could make it grow the line
  }

  /** Returns the unified visualizer for display. */
  public LoggedMechanism2d getVisualizer() {
    return visualizer;
  }

  /**
   * A high-level command to pick up a coral game piece. This command coordinates the three
   * components by: - Lowering the elevator to the intake position, - Rotating the arm into
   * position, - Activating the end effector to intake.
   *
   * @return a Command that runs the pickup sequence.
   */
  public Command pickupCoral() {
    return Commands.runOnce(
        () -> {
          elevator.setPosition(0.5); // Example: Lower elevator for intake.
          arm.setAngleDegrees(90); // Example: Rotate arm for intake.
          endEffector.runOpenLoop(1.0); // Activate end effector to intake.
        });
  }

  /**
   * A high-level command to drop off a coral game piece. This command coordinates the three
   * components by: - Raising the elevator to the dropoff position, - Rotating the arm to the
   * dropoff configuration, - Activating the end effector to eject.
   *
   * @return a Command that runs the dropoff sequence.
   */
  public Command dropoffCoral() {
    return Commands.runOnce(
        () -> {
          elevator.setPosition(1.5); // Example: Raise elevator for dropoff.
          arm.setAngleDegrees(30); // Example: Rotate arm for dropoff.
          endEffector.runOpenLoop(-1.0); // Activate end effector to eject.
        });
  }

  /**
   * A command that tells the arm to hold its current position. This is especially useful when
   * transitioning from disabled to teleop.
   *
   * @return a Command that holds the current arm position.
   */
  public Command holdArmPosition() {
    return Commands.runOnce(() -> arm.holdArmAngle());
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Arm getArm() {
    return arm;
  }

  public EndEffector getEndEffector() {
    return endEffector;
  }

  public boolean haveCoral() {
    return haveCoral;
  }

  public void setHaveCoral() {
    this.haveCoral = true;
  }

  public void setDoNotHaveCoral() {
    this.haveCoral = false;
  }

  public boolean haveAlgae() {
    return haveCoral;
  }

  public void setHaveAlgae() {
    this.haveCoral = true;
  }

  public void setDoNotHaveAlgae() {
    this.haveCoral = false;
  }
}
