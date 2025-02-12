package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 90, CoralState.STARTUP, 0.0, 0.2),
  STOW(22, 105, CoralState.STOW, 0.37, 0.2),
  PICKUP(36.0, 230, CoralState.PICKUP, 0.37, 0.2),
  SCORE_LEVEL_1(0.00000, 90, CoralState.L1_SCORE, 0.0, 0.2),
  SCORE_LEVEL_2(21.0, 70, CoralState.L2_SCORE, 0.37, 0.2),
  SCORE_LEVEL_3(37, 70, CoralState.L3_SCORE, 0.37, 0.2),
  SCORE_LEVEL_4(65, 55, CoralState.L4_SCORE, 0.37, 0.2),
  PREPARE_DISLODGE_LEVEL_1(15, 45, CoralState.PREP_L1_DISLODGE, 0.37, 0.2),
  FINAL_DISLODGE_LEVEL_1(22, 110, CoralState.FINAL_L1_DISLODGE, 0.37, 0.2),
  PREPARE_DISLODGE_LEVEL_2(25, 45, CoralState.PREP_L2_DISLODGE, 0.37, 0.2),
  FINAL_DISLODGE_LEVEL_2(32, 110, CoralState.FINAL_L2_DISLODGE, 0.37, 0.2);

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
    PREP_L1_DISLODGE,
    FINAL_L1_DISLODGE,
    PREP_L2_DISLODGE,
    FINAL_L2_DISLODGE,
  }
}
