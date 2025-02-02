package frc.robot.subsystems.coral.arm;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {

  public static double VISUALIZATION_CALIBRATION_OFFSET_DEGREES = 90.0;

  // Arm Motor Configuration Constants
  public static final int motorCanId = 2;
  public static final int armCurrentLimit = 50;
  public static final double kArmGearing = 100;
  public static final double kArmMomentOfInertia = 0.1;
  public static final double kArmMass = 4; // kg
  public static final double kArmLengthMeters = Units.inchesToMeters(15);

  public static final double kArmKp = .8;
  public static final double kArmKi = 0.0;
  public static final double kArmKd = 0.0;
  public static final double kArmKf = 0.0;
  public static final double kArmVel = 2000;
  public static final double kArmAcc = 4000;
  public static final double kArmMinAngleDegrees = -60;
  public static final double kArmMaxAngleDegrees = 240;
  public static final double kArmHomeAngleDegrees = 90;

  public static final double kArmkS = 0.2; // volts (V)
  public static final double kArmG = 0.2; // volts (V)
  public static final double kArmkV = 0.5; // volt per velocity (V/(m/s))
  public static final double kArmkA = 0.5; // volt per acceleration (V/(m/s²))

  private ArmConstants() {}
}
