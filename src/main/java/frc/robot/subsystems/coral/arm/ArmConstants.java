package frc.robot.subsystems.coral.arm;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {
  // Arm Motor Configuration Constants
  public static final int motorCanId = 2;
  public static final int armCurrentLimit = 50;
  public static final double kArmGearing = 100;
  public static final double kArmMomentOfInertia = 0.1;
  public static final double kArmMass = 4; // kg
  public static final double kArmLengthMeters = Units.inchesToMeters(7.529);

  public static final double kArmKp = 0.6;
  public static final double kArmKi = 0.0;
  public static final double kArmKd = 0.0;
  public static final double kArmKf = 0.0;
  public static final double kArmVel = 500;
  public static final double kArmAcc = 1000;
  public static final double kArmMinAngleDegrees = 0;
  public static final double kArmMaxAngleDegrees = 180;
  public static final double kArmHomeAngleDegrees = 90;

  public static final double kArmkS = 0.0; // volts (V)
  public static final double kArmG = 0.762; // volts (V)
  public static final double kArmkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kArmkA = 0.0; // volt per acceleration (V/(m/s²))

  private ArmConstants() {}
}
