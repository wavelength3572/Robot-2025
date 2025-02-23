package frc.robot.subsystems.coral.arm;

public class ArmConstants {
  public static final int canId = 15;
  public static final int armCurrentLimit = 50;
  public static final double kArmGearing = 48.0; // 16:1 (Motor Gearing) * 48:16 (Sprockets)

  public static final double armStartAngle = 105.0; // Degrees
  public static final double armMinAngle = 40.0; // Degrees
  public static final double armMaxAngle = 240.0; // Degrees
  public static final double armTBEOffset = 28.56; // Degrees

  public static final double kArmKp = .3;
  public static final double kArmKd = 0.00;
  public static final double kArmKfCoral = 0.21;
  public static final double kArmKfNoCoral = 0.162;
  public static final double kArmVel = 3000;
  public static final double kArmAcc = 5000;
  public static final double kAllowableError = .05;

  public static final double kArmkS = 0.0; // volts (V)
  public static final double kArmkG = 0.0; // volts (V)
  public static final double kArmkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kArmkA = 0.0; // volt per acceleration (V/(m/s²))

  public static final double kSetpointThresholdDEG = 1;
}
