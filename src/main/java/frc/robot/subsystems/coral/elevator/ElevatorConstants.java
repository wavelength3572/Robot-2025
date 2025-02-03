package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int leaderCanId = 21;
  public static final int followerCanId = 16;
  public static final int elevatorCurrentLimit = 50;
  public static final double kElevatorGearing = 9.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.25) / 2.0;
  public static final double kCarriageMass = 7.0; // kg

  public static final double kMinElevatorHeightMeters = 0.0; // m
  public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(60.748); // m

  public static final double kGroundToElevator = Units.inchesToMeters(10.5); // m

  public static final double kElevatorKp = .6;
  public static final double kElevatorKi = 0;
  public static final double kElevatorKd = 0.01;
  public static final double kElevatorKf = 0;
  public static final double kElevatorVel = 4500;
  public static final double kElevatorAcc = 13500;

  public static final double kElevatorkS = 0.0; // volts (V)
  public static final double kElevatorkG = 0.762; // volts (V)
  public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
}
