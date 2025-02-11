package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 90, CoralState.STARTUP, 0.0, 0.0),
  STOW(22, 100, CoralState.STOW, 0.0, 0.0),
  PICKUP(36.0, 230, CoralState.PICKUP, 0.0, 0.0),
  SCORE_LEVEL_1(0.00000, 90, CoralState.L1_SCORE, 0.0, 0.0),
  SCORE_LEVEL_2(21.0, 70, CoralState.L2_SCORE, 0.0, 0.0),
  SCORE_LEVEL_3(37, 70, CoralState.L3_SCORE, 0.0, 0.0),
  SCORE_LEVEL_4(65, 55, CoralState.L4_SCORE, 0.0, 0.0),
  DISLODGE_LEVEL_1(23.6220, 45, CoralState.L1_DISLODGE, 0.0, 0.0),
  DISLODGE_LEVEL_2(47.2440, 45, CoralState.L2_DISLODGE, 0.0, 0.0);
  // SCORE_LEVEL_2_Straight(Units.inchesToMeters(12), 90, CoralState.PREPARE_L2_SCORE),
  // SCORE_LEVEL_3_Straight(Units.inchesToMeters(), 90, CoralState.PREPARE_L2_SCORE),
  // highest height 65 inches

  @Getter private final double elevatorHeight;
  @Getter private final double armAngle;
  @Getter private final CoralState state;
  @Getter private final double elevatorFF;
  @Getter private final double armFF;

  CoralSystemPresets(
      double elevatorHeightINCHES,
      double armAngleDEGREES,
      CoralState state,
      double elevatorFF,
      double armFF) {
    this.elevatorHeight = elevatorHeightINCHES;
    this.armAngle = armAngleDEGREES;
    this.state = state;
    this.elevatorFF = elevatorFF;
    this.armFF = armFF;
  }

  public enum CoralState {
    STARTUP,
    STOW,
    PICKUP,
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    L1_DISLODGE,
    L2_DISLODGE
  }
}
