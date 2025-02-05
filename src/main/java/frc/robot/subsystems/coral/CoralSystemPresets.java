package frc.robot.subsystems.coral;

import edu.wpi.first.math.util.Units;

public enum CoralSystemPresets {
  STOW(0, 90, CoralState.PREPARE_STOW),
  STOW_LOW(.52, 235, CoralState.PREPARE_STOW_LOW),
  SAFE_CARRIAGE_POSITION(.52, Double.NaN, CoralState.PREPARE_SAFE_CARRIAGE_POSITION),

  PICKUP(0.826, 230, CoralState.PICKUP),

  SCORE_LEVEL_1(Units.inchesToMeters(10.2362), 45, CoralState.PREPARE_L1_SCORE),
  SCORE_LEVEL_2(Units.inchesToMeters(18.8976), 47, CoralState.PREPARE_L2_SCORE),
  SCORE_LEVEL_3(Units.inchesToMeters(36.61417), 47, CoralState.PREPARE_L3_SCORE),
  SCORE_LEVEL_4(Units.inchesToMeters(59.0551), 45, CoralState.PREPARE_L4_SCORE),

  // SCORE_LEVEL_2_Straight(Units.inchesToMeters(12), 90, CoralState.PREPARE_L2_SCORE),
  // SCORE_LEVEL_3_Straight(Units.inchesToMeters(), 90, CoralState.PREPARE_L2_SCORE),
  // highest height 65 inches

  // Algae dislodge positions
  FRONT_ALGAE_DISLODGE_LEVEL_1(
      Units.inchesToMeters(23.622), 45, CoralState.PREPARE_FRONT_ALGAE_DISLODGE),
  FRONT_ALGAE_DISLODGE_LEVEL_2(
      Units.inchesToMeters(47.2441), 85, CoralState.PREPARE_FRONT_ALGAE_DISLODGE),
  BACK_ALGAE_DISLODGE_LEVEL_1(
      Units.inchesToMeters(23.622), 195, CoralState.PREPARE_BACK_ALGAE_DISLODGE),
  BACK_ALGAE_DISLODGE_LEVEL_2(
      Units.inchesToMeters(39.3701), 195, CoralState.PREPARE_BACK_ALGAE_DISLODGE);

  private final double elevatorHeight;
  private final double armAngle;
  private final CoralState state;

  CoralSystemPresets(double elevatorHeight, double armAngle, CoralState state) {
    this.elevatorHeight = elevatorHeight;
    this.armAngle = armAngle;
    this.state = state;
  }

  public double getElevatorHeight() {
    return elevatorHeight;
  }

  public double getArmAngle() {
    return armAngle;
  }

  public CoralState getState() {
    return state;
  }

  public enum CoralState {
    // Prepare states (Moving towards final states)
    PREPARE_STOW,
    PREPARE_STOW_LOW,
    PREPARE_SAFE_CARRIAGE_POSITION,
    PREPARE_CORAL_STATION_PICKUP,
    PREPARE_L1_SCORE,
    PREPARE_L2_SCORE,
    PREPARE_L3_SCORE,
    PREPARE_L4_SCORE,
    PREPARE_FRONT_ALGAE_DISLODGE,
    PREPARE_BACK_ALGAE_DISLODGE,

    // Final states (Robot is fully in position)
    STOW,
    STOW_LOW,
    SAFE_CARRIAGE_POSITION,
    PICKUP,
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    FRONT_ALGAE_DISLODGE,
    BACK_ALGAE_DISLODGE
  }
}
