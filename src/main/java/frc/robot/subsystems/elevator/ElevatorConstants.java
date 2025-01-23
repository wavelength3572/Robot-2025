package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int leaderCanId = 99;
  public static final int followerCanId = 98;
  public static final int currentLimit = 40;
  public static final double kElevatorGearing = 20.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.25) / 2.0;
  public static final double kCarriageMass = 7.0; // kg

  public static final double kMinElevatorHeightMeters = 0.0; // m
  public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(60.748); // m

  public static final double kGroundToElevator = Units.inchesToMeters(7.529); // m

  public static final double kElevatorKp = 5;
  public static final double kElevatorKi = 0;
  public static final double kElevatorKd = 0;

  public static final double kElevatorkS = 0.0; // volts (V)
  public static final double kElevatorkG = 0.762; // volts (V)
  public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
}
