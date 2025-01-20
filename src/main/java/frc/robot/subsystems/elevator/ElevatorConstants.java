package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int ElevatorCanId = 99;
  public static final double motorReduction = 11.0 / 1630; // Rotations / Max Height in mm
  public static final int currentLimit = 40;
  public static final double kElevatorGearing = 20.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.25) / 2.0;
  public static final double kCarriageMass = 7.0; // kg

  public static final double kPixelsPerMeter = 20;

  public static final double kMinElevatorHeightMeters = 0.0; // m
  public static final double kMaxElevatorHeightMeters = Units.feetToMeters(6.0); // m
}
