package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.endeffector.EndEffector;

public class CoralSubsystem extends SubsystemBase {
  // Subcomponents that handle their own I/O and control logic
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;

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
  }

  /** The periodic method updates all subcomponents. */
  @Override
  public void periodic() {

    elevator.update();
    arm.update();
    endEffector.update();
  }

  /**
   * A high-level command to pick up a coral game piece. This command coordinates the three
   * components by: - Lowering the elevator to the intake position, - Rotating the arm into
   * position, - Activating the end effector (with positive output) to intake.
   *
   * @return a Command that runs the pickup sequence.
   */
  public Command pickupCoral() {
    return Commands.runOnce(
        () -> {
          elevator.setPosition(0.5); // Example: Lower elevator for intake.
          arm.setAngle(0.3); // Example: Rotate arm for intake.
          endEffector.runOpenLoop(1.0); // Activate end effector to intake.
        });
  }

  /**
   * A high-level command to drop off a coral game piece. This command coordinates the three
   * components by: - Raising the elevator to the dropoff position, - Rotating the arm to the
   * dropoff configuration, - Activating the end effector (with negative output) to eject.
   *
   * @return a Command that runs the dropoff sequence.
   */
  public Command dropoffCoral() {
    return Commands.runOnce(
        () -> {
          elevator.setPosition(1.5); // Example: Raise elevator for dropoff.
          arm.setAngle(1.0); // Example: Rotate arm for dropoff.
          endEffector.runOpenLoop(-1.0); // Activate end effector to eject.
        });
  }

  // Getters for the subcomponents, in case they are needed elsewhere.
  public Elevator getElevator() {
    return elevator;
  }

  public Arm getArm() {
    return arm;
  }

  public EndEffector getEndEffector() {
    return endEffector;
  }
}
