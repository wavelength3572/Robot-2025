package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int canId = 99;
  public static final int armCurrentLimit = 50;
  public static final double kArmGearing = 15.0; // 5:1 (Motor Gearing) * 48:16 (Sprockets)

  public static final double kArmKp = .2;
  public static final double kArmKd = 0.01;
  public static final double kArmKf = 0;

  public static final double kArmkS = 0.0; // volts (V)
  public static final double kArmkG = 0.762; // volts (V)
  public static final double kArmkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kArmkA = 0.0; // volt per acceleration (V/(m/s²))
}
