package frc.robot.subsystems.coral.endeffector;

/** EndEffectorConstants defines hardware-specific constants for the end effector. */
public final class EndEffectorConstants {
  // The CAN ID for the end effector motor.
  public static final int motorCanId = 5; // Change this to your actual CAN ID.

  // The maximum voltage to be applied.
  public static final double maxVoltage = 12.0;

  // Whether the motor should be inverted.
  public static final boolean invertMotor = false;

  private EndEffectorConstants() {
    // Prevent instantiation.
  }
}
