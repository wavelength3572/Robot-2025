package frc.robot.subsystems.coral.arm;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {

  public static double armLength = Units.inchesToMeters(7.529);

  // Example CAN ID for the arm motor.
  public static final int motorCanId = 2;

  // PID constants for the arm simulation.
  public static final double kArmKp = 0.5;
  public static final double kArmKi = 0.0;
  public static final double kArmKd = 0.0;

  // Maximum angular velocity and acceleration in rotations per second and rotations per second^2.
  // For example, if your arm is expected to move at 90°/s, that's 0.25 rotations per second.
  public static final double kMaxAngularVelocityRotations = 0.25;
  public static final double kMaxAngularAccelerationRotations = 0.5;

  private ArmConstants() {}
}
