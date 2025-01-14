package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int ElevatorCanId = 99;
  public static final double motorReduction = 11.0 / 1630; // Rotations / Max Height in mm
  public static final int currentLimit = 40;
  public static final double kElevatorGearing = 10.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  public static final double kCarriageMass = 4.0; // kg
}
